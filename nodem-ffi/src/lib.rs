#![no_std]

extern crate alloc;

use alloc::boxed::Box;
use core::ffi::c_void;
use core::fmt::Write;
use core::slice;

use embedded_alloc::LlffHeap as Heap;
use nodem_rs::media;
use nodem_rs::runtime::{Runtime, DOM};

mod control;
use control::ZephyrControl;

// `nodem-rs` (`alloc` feature) needs a global allocator; since this staticlib
// is the only Rust code in the final Zephyr link, nothing else can provide
// one.
#[global_allocator]
static HEAP: Heap = Heap::empty();

const HEAP_SIZE: usize = 32 * 1024;
static mut HEAP_MEM: [u8; HEAP_SIZE] = [0; HEAP_SIZE];
static mut HEAP_INITIALIZED: bool = false;

fn ensure_heap() {
    unsafe {
        if !HEAP_INITIALIZED {
            HEAP.init((&raw mut HEAP_MEM) as usize, HEAP_SIZE);
            HEAP_INITIALIZED = true;
        }
    }
}

// `nodem_process_command` is only ever called from one thread (nodem_task,
// sequentially - see zephyr-app/src/nodem_task.c), so a single global
// instance shared across calls (rather than a fresh one per call, which
// would lose e.g. a "#reset" flag before anything could act on it) needs no
// synchronization beyond that.
static mut CONTROL: ZephyrControl = ZephyrControl::new();

// nodem-rs logs internally via `log::info!`/`log::error!`; without a logger
// installed those are silent no-ops. Forward them to the same UART console
// as everything else, via a small C bridge (nodem_log_write, implemented in
// zephyr-app/src/nodem_task.c) rather than calling printk() directly - Rust
// can't call a C variadic function.
unsafe extern "C" {
    fn nodem_log_write(level_ptr: *const u8, level_len: usize, msg_ptr: *const u8, msg_len: usize);
}

// pkg_store.c's raw, memory-mapped read accessor for the "pkg" partition -
// see nodem_load_pkg() below. Safe to dereference directly (no driver call,
// no copy) because pm_static.yml also declares a "nonsecure_storage"
// partition covering the same address range - see pkg_store_data()'s doc
// comment in pkg_store.h for how that grants the SPU access this needs.
unsafe extern "C" {
    fn pkg_store_data(len: *mut usize) -> *const u8;
}

const LOG_BUF_SIZE: usize = 128;

/// Fixed-size, truncating `core::fmt::Write` buffer for formatting one log
/// line before handing it to `nodem_log_write`.
struct LogBuf {
    buf: [u8; LOG_BUF_SIZE],
    len: usize,
}
impl Write for LogBuf {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let bytes = s.as_bytes();
        let remaining = LOG_BUF_SIZE - self.len;
        let n = bytes.len().min(remaining);
        self.buf[self.len..self.len + n].copy_from_slice(&bytes[..n]);
        self.len += n;
        Ok(())
    }
}

struct FfiLogger;
static LOGGER: FfiLogger = FfiLogger;

impl log::Log for FfiLogger {
    fn enabled(&self, _metadata: &log::Metadata) -> bool {
        true
    }

    fn log(&self, record: &log::Record) {
        let level = record.level().as_str();
        let mut msg = LogBuf { buf: [0; LOG_BUF_SIZE], len: 0 };
        let _ = write!(msg, "{}", record.args());
        unsafe { nodem_log_write(level.as_ptr(), level.len(), msg.buf.as_ptr(), msg.len) };
    }

    fn flush(&self) {}
}

static mut LOGGER_INITIALIZED: bool = false;

fn ensure_logger() {
    unsafe {
        if !LOGGER_INITIALIZED {
            // `set_logger` only allowed once per process - fine, since
            // `nodem_runtime_new` (the only caller) is itself only ever
            // called once.
            let _ = log::set_logger(&LOGGER);
            log::set_max_level(log::LevelFilter::Info);
            LOGGER_INITIALIZED = true;
        }
    }
}

/// Truncating `core::fmt::Write` adapter over a caller-owned output buffer.
struct OutputWriter<'a> {
    buf: &'a mut [u8],
    len: usize,
}
impl<'a> Write for OutputWriter<'a> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let bytes = s.as_bytes();
        let remaining = self.buf.len() - self.len;
        let n = bytes.len().min(remaining);
        self.buf[self.len..self.len + n].copy_from_slice(&bytes[..n]);
        self.len += n;
        Ok(())
    }
}

/// Creates a new nodem runtime rendering into `fb_ptr`/`fb_len`, a
/// caller-owned 1bpp framebuffer (`nodem_rs::SCREEN_BUFFER_SIZE` bytes for a
/// `width` x `height` panel) that must outlive the returned handle.
#[unsafe(no_mangle)]
pub extern "C" fn nodem_runtime_new(fb_ptr: *mut u8, fb_len: usize, width: u16, height: u16) -> *mut c_void {
    ensure_heap();
    ensure_logger();
    let fb = unsafe { slice::from_raw_parts_mut(fb_ptr, fb_len) };
    let dom = Box::new(DOM::new(fb, width, height));
    Box::into_raw(dom) as *mut c_void
}

/// Loads whatever's in the "pkg" flash partition (pkg_store.c) into
/// `handle`'s `Media`, if it holds a valid nodem-rs package. Call once at
/// boot, right after the *first* `nodem_runtime_run()` - not right after
/// `nodem_runtime_new`, before any render has happened: that first `run()`
/// is nodem's own initialization (it loads nodem-rs's built-in PKG_SYS as
/// source 0), and this needs to land after that, not race it. Call again
/// any time `nodem_control_take_pkg_ready()` returns true, so a
/// freshly-uploaded package takes effect immediately rather than only on
/// the next reboot. A first-boot device with nothing ever uploaded there
/// (an erased partition, all 0xFF) is the normal case, not an error, and is
/// silently skipped once its magic bytes fail to match.
///
/// Points `Media` directly at the "pkg" partition's own memory-mapped
/// address, no copy - safe because pm_static.yml declares a
/// "nonsecure_storage" partition covering the same range, which grants the
/// SPU access this needs (see pkg_store_data()'s doc comment in
/// pkg_store.h). `Media::load_pkg` stores raw pointers into whatever buffer
/// it's given, read again at render time, not just while it itself parses -
/// so this only works because that whole range is genuinely,
/// permanently Non-Secure, not just for the duration of this one call.
///
/// The package's own 4-byte length field (right after its "PKG0" magic) is
/// read here, up front, specifically so `Media::load_pkg` is handed a slice
/// sized to exactly the real package - not the whole partition - since its
/// CRC check covers every byte of whatever slice it's given; passing the
/// full (mostly-erased) partition would make that check fail even for a
/// perfectly valid, shorter package.
///
/// Returns whether a package was actually loaded.
#[unsafe(no_mangle)]
pub extern "C" fn nodem_load_pkg(handle: *mut c_void) -> bool {
    let dom = unsafe { &mut *(handle as *mut DOM) };

    let mut capacity: usize = 0;
    let ptr = unsafe { pkg_store_data(&raw mut capacity) };
    let data = unsafe { slice::from_raw_parts(ptr, capacity) };

    if capacity < 12 || &data[0..4] != b"PKG0" {
        log::info!("no valid pkg in 'pkg' partition");
        return false;
    }

    // This 4-byte field is the *total* package size, header included - the
    // same thing `PKG_SYS.len()` (nodem-rs core's own built-in package,
    // loaded the same way from a plain `include_bytes!`) already means when
    // passed to `load_pkg`, not a payload-only length to add 12 to. Getting
    // this wrong doesn't fail loudly: `load_pkg`'s CRC check covers
    // whatever slice length it's given, from byte 12 to the end, with no
    // reference to this field at all - passing 12 bytes too many (as a
    // `12 + length` reading of it did here before) silently pulls in
    // trailing erased-flash bytes past the real package and always fails
    // that check, even for a perfectly valid upload.
    let length = u32::from_le_bytes([data[4], data[5], data[6], data[7]]) as usize;

    // `load_pkg` itself indexes `data[12..]` unconditionally once past the
    // magic/length check - below 12, that's a panic, not a clean error, so
    // this has to be validated here rather than left to it.
    if length < 12 || length > capacity {
        log::error!("'pkg' partition: declared length {length}b invalid (capacity {capacity}b)");
        return false;
    }

    // Source 1: a permanently (flash-)stored package. Source 0 is
    // nodem-rs's own built-in PKG_SYS, loaded separately by DOM::run() on
    // its first frame; source 2 is nodem-rs core's own default
    // IControlLoader (control/media.rs's `impl IControlLoader for Surface`,
    // backed by a RAM-only buffer) - not this one, which persists across
    // reboots.
    let ret = dom.surface.media.load_pkg(ptr, length, 1);
    match ret {
        media::Ret::Ok => {
            log::info!("loaded pkg from flash [{length}b]");
            true
        }
        _ => {
            log::error!("pkg in flash failed to load: {}", ret as u8);
            false
        }
    }
}

/// Advances the runtime by one frame, rendering into the framebuffer passed
/// to `nodem_runtime_new`. Call periodically and hand the same framebuffer to
/// the display driver afterward.
#[unsafe(no_mangle)]
pub extern "C" fn nodem_runtime_run(handle: *mut c_void) -> bool {
    let dom = unsafe { &mut *(handle as *mut DOM) };
    dom.run()
}

/// Feeds one command line into the runtime and writes its text response into
/// `output_ptr`/`output_cap`, storing the number of bytes written in
/// `*output_len`. Returns the number of input bytes consumed.
#[unsafe(no_mangle)]
pub extern "C" fn nodem_process_command(
    handle: *mut c_void,
    input_ptr: *const u8,
    input_len: usize,
    output_ptr: *mut u8,
    output_cap: usize,
    output_len: *mut usize,
) -> usize {
    let dom = unsafe { &mut *(handle as *mut DOM) };
    let input = unsafe { slice::from_raw_parts(input_ptr, input_len) };
    let output = unsafe { slice::from_raw_parts_mut(output_ptr, output_cap) };

    let mut writer = OutputWriter { buf: output, len: 0 };
    let control = unsafe { &mut *(&raw mut CONTROL) };
    let (consumed, _) = dom.process_command(input, &mut writer, control);

    unsafe { *output_len = writer.len };
    consumed
}

/// See nodem_ffi.h's doc comment on `nodem_control_take_restart` (declared
/// there, for nodem_task.c to call after flushing `nodem_process_command`'s
/// response).
#[unsafe(no_mangle)]
pub extern "C" fn nodem_control_take_restart() -> bool {
    let control = unsafe { &mut *(&raw mut CONTROL) };
    control.take_restart()
}

/// See nodem_ffi.h's doc comment on `nodem_control_take_pkg_ready` (declared
/// there, for nodem_task.c to call `nodem_load_pkg()` again once it returns
/// true, making a just-uploaded package take effect immediately).
#[unsafe(no_mangle)]
pub extern "C" fn nodem_control_take_pkg_ready() -> bool {
    let control = unsafe { &mut *(&raw mut CONTROL) };
    control.take_pkg_ready()
}

// `k_panic()`/`irq_lock()`/`irq_unlock()` are C macros/static-inline
// functions with no linkable symbol - call the real exported kernel
// functions they resolve to instead.
unsafe extern "C" {
    fn k_fatal_halt(reason: u32) -> !;
    fn k_sched_lock();
    fn k_sched_unlock();
}

// K_ERR_KERNEL_PANIC from zephyr/include/zephyr/fatal_types.h's
// `enum k_fatal_error_reason` (not usable directly from Rust - no header).
const K_ERR_KERNEL_PANIC: u32 = 4;

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    unsafe { k_fatal_halt(K_ERR_KERNEL_PANIC) }
}

// `embedded-alloc`'s heap lock (and anything else in this crate's dependency
// tree using `critical-section`) needs a backend. All allocation here
// happens from normal Zephyr thread context (never an ISR), so a scheduler
// lock - preventing preemption mid-allocation - is the right level of
// protection, rather than a full IRQ mask.
struct ZephyrCriticalSection;
critical_section::set_impl!(ZephyrCriticalSection);

unsafe impl critical_section::Impl for ZephyrCriticalSection {
    unsafe fn acquire() -> u32 {
        unsafe { k_sched_lock() };
        0
    }

    unsafe fn release(_token: u32) {
        unsafe { k_sched_unlock() };
    }
}
