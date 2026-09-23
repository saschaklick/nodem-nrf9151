use core::ffi::c_char;

use nodem_rs::control::{Control, ControlMode, IControl, IControlLoader, LoaderRet};
use nodem_rs::media::Media;

// C ABI this bridges to - config_store.c (flash-backed key/value store),
// modem_task.c (LTE/TLS/cloud-registration status), and pkg_store.c (raw
// "pkg" flash partition - see IControlLoader impl below). See those files'
// headers for the full doc comments. nodem_listener.c's actual
// nodem_hw_reset() is deliberately *not* called from here - see `restart`'s
// doc comment below.
unsafe extern "C" {
    fn config_set_str(key: *const c_char, val: *const c_char) -> i32;
    fn config_list(buf: *mut c_char, buf_len: usize) -> usize;
    fn modem_status_format(buf: *mut c_char, buf_len: usize) -> usize;
    fn pkg_store_capacity() -> usize;
    fn pkg_store_erase(len: usize) -> i32;
    fn pkg_store_write(offset: usize, buf: *const u8, len: usize) -> i32;
}

// Matches pkg_store.h's PKG_STORE_MAX_CHUNK - pkg_store_write() rejects
// anything longer than this in one call.
const PKG_CHUNK_LEN: usize = 512;

// Matches config_store.h's MAX_KEY_LEN/MAX_STR_LEN (+1 for the NUL each C
// call needs) - kept as separate constants here since control.rs has no way
// to `#include` that header and assert they match at compile time.
const KEY_BUF_LEN: usize = 16;
const VAL_BUF_LEN: usize = 128;

// Matches nodem-esp32's websocket.rs/command_listener.rs - same cloud API,
// same registration code/device name shape (see modem_task.c's
// step_register(), which actually performs the registration request this
// provisions).
const REG_CODE_LEN: usize = 6;
const DEVICE_NAME_MIN_LEN: usize = 3;
const DEVICE_NAME_MAX_LEN: usize = 64;

/// Copies `s` into `buf` NUL-terminated, for passing to a C call. Fails
/// (rather than silently truncating, which could target/store the wrong
/// key) if `s` doesn't fit or already contains a NUL.
fn to_cstr<'a>(s: &str, buf: &'a mut [u8]) -> Option<&'a [u8]> {
    if s.len() >= buf.len() || s.as_bytes().contains(&0) {
        return None;
    }
    buf[..s.len()].copy_from_slice(s.as_bytes());
    buf[s.len()] = 0;
    Some(&buf[..=s.len()])
}

#[repr(u8)]
#[derive(PartialEq)]
enum Ret {
    Ok = 0,
    Error = 1,
    MalformedValue = 2,
}

/// The command handler `Runtime::process_command` invokes for each complete
/// `#...` line over UART - bridges nodem-rs's transport-agnostic command
/// protocol to this firmware's actual hardware. Currently: "#reg" to
/// provision cloud registration (config_store.c's flash-backed key/value
/// store; modem_task.c's step_register() does the actual work), "#nvs" to
/// list that store's contents, "#stat" against modem_task.c's connection
/// status, and "#reset". More of "controlling all aspects of NRF hardware"
/// is meant to grow the same way - a small C bridge function (see
/// nodem_listener.c) plus a case here.
///
/// Deliberately doesn't reset synchronously from `process_line` - the reply
/// this produces has to actually reach the caller over UART before the
/// reset happens, so `restart` is only a flag here; `take_restart()`
/// (called from nodem_task.c, via `nodem_control_take_restart` in lib.rs,
/// only after the response has been written out) is what acts on it.
pub struct ZephyrControl {
    restart: bool,
    // Backing store for "pkg" uploads (see the IControlLoader impl below) -
    // same shape as nodem-esp32's command_listener.rs: bytes are buffered
    // here up to PKG_CHUNK_LEN before being flushed to pkg_store_write() in
    // one call, since neither pkg_store.c's write-block padding nor a flash
    // driver in general wants to be called one byte at a time.
    pkg_buf: [u8; PKG_CHUNK_LEN],
    pkg_buf_len: usize,
    // Absolute offset in the "pkg" partition of the next byte to be written -
    // process_loader_end() needs this to flush a final, sub-PKG_CHUNK_LEN
    // chunk, since (unlike process_loader_data()) it isn't given a position.
    pkg_written: usize,
    // Set by process_loader_end() once a package has been fully flashed -
    // *not* itself a call to Media::load_pkg(), since IControlLoader gives
    // process_loader_end() no way to reach the live Surface/Media (unlike
    // nodem-esp32's own equivalent, which implements this trait directly on
    // Surface). Same deferred shape as `restart`/nodem-esp32's own
    // `pkg_reload`: take_pkg_ready() (via nodem_control_take_pkg_ready() in
    // lib.rs) is polled by nodem_task.c, which re-runs nodem_load_pkg() -
    // the exact function that already loads the "pkg" partition at boot -
    // so an upload takes effect right away instead of only on next reboot.
    pkg_ready: bool,
}

impl ZephyrControl {
    pub const fn new() -> Self {
        Self {
            restart: false,
            pkg_buf: [0u8; PKG_CHUNK_LEN],
            pkg_buf_len: 0,
            pkg_written: 0,
            pkg_ready: false,
        }
    }

    pub fn take_restart(&mut self) -> bool {
        core::mem::take(&mut self.restart)
    }

    pub fn take_pkg_ready(&mut self) -> bool {
        core::mem::take(&mut self.pkg_ready)
    }
}

impl IControl for ZephyrControl {
    fn process_line(
        &mut self,
        line: &str,
        _media: &Media,
        res: &mut dyn core::fmt::Write,
    ) -> (bool, core::fmt::Result) {
        let prefix = "#";
        if !line.starts_with(prefix) {
            return (false, Ok(()));
        }

        let mut split = line[prefix.len()..].splitn(2, ',');
        let command = split.next().unwrap_or("").trim();
        let args = split.next().unwrap_or("");
        let mut ret = Ret::Ok;

        match command {
            "reset" => {
                self.restart = true;
            }
            // "#reg,<code>,<name>" - provisions a cloud registration code
            // and device name. Both args are validated *before* either is
            // written, same reasoning as nodem-esp32's command_listener.rs:
            // a naive "store code, then store name" order can leave a
            // too-short/too-long name alone persisted after the code write
            // already succeeded, and modem_task.c's step_register() would
            // then see a pending code with an invalid name every retry.
            "reg" => {
                let mut parts = args.splitn(2, ',');
                let code = parts.next().unwrap_or("").trim();
                let name = parts.next().unwrap_or("").trim();

                if code.len() != REG_CODE_LEN
                    || !(DEVICE_NAME_MIN_LEN..=DEVICE_NAME_MAX_LEN).contains(&name.len())
                {
                    ret = Ret::MalformedValue;
                } else {
                    let mut code_buf = [0u8; KEY_BUF_LEN];
                    let mut name_buf = [0u8; VAL_BUF_LEN];

                    match (to_cstr(code, &mut code_buf), to_cstr(name, &mut name_buf)) {
                        (Some(c), Some(n)) => {
                            let err_code = unsafe {
                                config_set_str(c"reg_code".as_ptr(), c.as_ptr() as *const c_char)
                            };
                            let err_name = unsafe {
                                config_set_str(c"device_name".as_ptr(), n.as_ptr() as *const c_char)
                            };
                            if err_code != 0 || err_name != 0 {
                                ret = Ret::Error;
                            }
                        }
                        _ => ret = Ret::MalformedValue,
                    }
                }
            }
            // "#host,<host>" - overrides the cloud host modem_task.c
            // connects to (config_store's "cloud_host", read by its
            // get_cloud_host()); a bare "#host" (empty value) clears back
            // to the built-in default. Either way also clears
            // "device_id"/"device_secret" - a registration is only
            // meaningful against whichever host issued it, so switching
            // hosts must not let a stale id/secret get reused against the
            // new one. Matches nodem-esp32's own "#host" exactly, including
            // that this alone doesn't trigger a fresh registration attempt:
            // modem_task.c's step_register() only acts once "#reg"
            // provisions a new code.
            "host" => {
                let host = args.trim();

                let store_err = if host.is_empty() {
                    let empty: [u8; 1] = [0];
                    unsafe { config_set_str(c"cloud_host".as_ptr(), empty.as_ptr() as *const c_char) }
                } else {
                    let mut host_buf = [0u8; VAL_BUF_LEN];
                    match to_cstr(host, &mut host_buf) {
                        Some(h) => unsafe {
                            config_set_str(c"cloud_host".as_ptr(), h.as_ptr() as *const c_char)
                        },
                        None => {
                            ret = Ret::MalformedValue;
                            0
                        }
                    }
                };

                if ret == Ret::Ok {
                    if store_err != 0 {
                        ret = Ret::Error;
                    } else {
                        let empty: [u8; 1] = [0];
                        unsafe {
                            let _ = config_set_str(c"device_id".as_ptr(), empty.as_ptr() as *const c_char);
                            let _ =
                                config_set_str(c"device_secret".as_ptr(), empty.as_ptr() as *const c_char);
                        }
                    }
                }
            }
            "nvs" => {
                let mut buf = [0u8; 512];
                let n = unsafe { config_list(buf.as_mut_ptr() as *mut c_char, buf.len()) };
                let n = n.min(buf.len());
                let text = core::str::from_utf8(&buf[..n]).unwrap_or("");
                let _ = res.write_str(text);
            }
            "stat" => {
                let mut buf = [0u8; 224];
                let n = unsafe { modem_status_format(buf.as_mut_ptr() as *mut c_char, buf.len()) };
                let n = n.min(buf.len() - 1);
                let status = core::str::from_utf8(&buf[..n]).unwrap_or("");
                let _ = write!(res, "modem,{status}\r\n");
            }
            _ => ret = Ret::Error,
        }

        Control::send_result(prefix, ret as u8, res)
    }

    fn get_loader(&mut self) -> Option<&mut dyn IControlLoader> {
        Some(self)
    }
}

/// Streams a "pkg" upload - a nodem-rs binary UI/Media package, same format
/// and same upload protocol as nodem-esp32, *not* a firmware image - straight
/// into the "pkg" flash partition (pkg_store.c/pm_static.yml): erase up
/// front in `process_loader_start`, forward-only writes in
/// `process_loader_data`, flush the final partial chunk in
/// `process_loader_end`. Ported from nodem-esp32's command_listener.rs, with
/// pkg_store.c's flash_area calls standing in for ESP-IDF's `EspPartition`.
/// Unlike that project, there's no mmap step needed here to make the written
/// bytes loadable - the nRF9151's internal flash is already directly
/// addressable at its physical address (PM_PKG_ADDRESS), no explicit mapping
/// call required. Actually loading a fully-received package into the
/// running DOM/Surface is nodem-rs's own concern, not this bridge's.
impl IControlLoader for ZephyrControl {
    fn process_loader_start(&mut self, mode: ControlMode, len: usize) -> usize {
        match mode {
            ControlMode::PKGMode => {
        
                log::info!("pkg loader start: {len}b");
                self.pkg_buf_len = 0;
                self.pkg_written = 0;

                let capacity = unsafe { pkg_store_capacity() };
                if capacity < len {
                    log::error!("'pkg' partition ({capacity}b) too small for {len}b upload");
                    return 0;
                }

                if unsafe { pkg_store_erase(len) } != 0 {
                    log::error!("'pkg' partition erase failed");
                    return 0;
                }

                capacity
            }
            _ => 0
        }
    }

    fn process_loader_data(&mut self, buf: &[u8], pos: usize) {
        for (i, &byte) in buf.iter().enumerate() {
            self.pkg_buf[self.pkg_buf_len] = byte;
            self.pkg_buf_len += 1;

            if self.pkg_buf_len == PKG_CHUNK_LEN {
                self.pkg_buf_len = 0;

                let offset = pos + i + 1 - PKG_CHUNK_LEN;
                let err = unsafe { pkg_store_write(offset, self.pkg_buf.as_ptr(), PKG_CHUNK_LEN) };
                if err != 0 {
                    log::error!("'pkg' partition write at {offset} failed: {err}");
                }
                self.pkg_written = offset + PKG_CHUNK_LEN;
            }
        }
    }

    fn process_loader_end(&mut self) -> LoaderRet {
        log::info!("pkg loader end: {}b", self.pkg_written + self.pkg_buf_len);

        if self.pkg_buf_len > 0 {
            let err =
                unsafe { pkg_store_write(self.pkg_written, self.pkg_buf.as_ptr(), self.pkg_buf_len) };
            if err != 0 {
                log::error!("'pkg' partition write at {} failed: {err}", self.pkg_written);
                self.pkg_buf_len = 0;
                return LoaderRet::PKGFailed;
            }
            self.pkg_written += self.pkg_buf_len;
            self.pkg_buf_len = 0;
        }

        self.pkg_ready = true;
        LoaderRet::Ok
    }
}
