use alloc::boxed::Box;
use alloc::format;
use defmt::info;

use embassy_nrf::buffered_uarte::BufferedUarteTx;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, Timer};

use nodem_rs::control::IControl;
use nodem_rs::media::Identifier;
use nodem_rs::runtime::{Runtime as _, DOM};
use nodem_rs::{Area, Point, Size};

pub const DISPLAY_WIDTH: u32 = 128;
pub const DISPLAY_HEIGHT: u32 = 64;
const BUFFER_LEN: usize = (DISPLAY_WIDTH * DISPLAY_HEIGHT / 8) as usize;

/// Staging buffer `uart_task` appends incoming bytes into; `nodem_task`
/// drains whatever complete commands have accumulated here each frame.
/// Sized like the ESP32 original's `command_buf`.
pub const COMMAND_BUF_LEN: usize = 1024;
const RESPONSE_BUF_LEN: usize = 1024;

/// `process_command` needs an `IControl` to dispatch device-specific "#..."
/// extension commands to (ESP32's `CommandListener` handles wifi/cloud/iled/
/// pkg-upload commands this way) - there's no such extension surface here
/// yet, so every default (`process_line` replies "?", no loader), leaving
/// only the commands `nodem_rs::control::Control` itself understands
/// (e.g. "info") reachable.
struct NoopControl;
impl IControl for NoopControl {}

/// Adapts a plain `&mut [u8]` + cursor into `core::fmt::Write` so
/// `process_command`'s response text can be captured into a stack buffer
/// instead of needing direct access to the UART hardware at the point
/// commands are processed. Silently truncates on overflow rather than
/// returning `Err` - some of `nodem_rs::control::Control`'s own command
/// handlers `.expect()` a successful `write_str`, so an `Err` here would
/// panic the firmware rather than just losing the tail of a reply.
struct BufWriter<'a> {
    buf: &'a mut [u8],
    pos: usize,
}
impl core::fmt::Write for BufWriter<'_> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let bytes = s.as_bytes();
        let n = bytes.len().min(self.buf.len() - self.pos);
        self.buf[self.pos..self.pos + n].copy_from_slice(&bytes[..n]);
        self.pos += n;
        if n < bytes.len() {
            defmt::warn!("command response truncated, response buffer full");
        }
        Ok(())
    }
}

/// How long the cloud status overlay stays visible after connecting, before
/// being hidden - mirrors the wifi+cloud "both connected" hide timer from the
/// original ESP32 `nodem_task`, now driven by cloud status alone since this
/// board has no Wi-Fi.
const HIDE_REPORT_AFTER: Duration = Duration::from_secs(5);

#[derive(Clone, Copy, PartialEq)]
pub enum CloudConnectionStatus {
    Disconnected,
    Connecting,
    Connected,
}

pub struct CloudStatus {
    pub connection: CloudConnectionStatus,
}

impl CloudStatus {
    const fn new() -> Self {
        Self { connection: CloudConnectionStatus::Disconnected }
    }
}

impl core::fmt::Display for CloudStatus {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self.connection {
            CloudConnectionStatus::Disconnected => write!(f, "Cloud: disconnected"),
            CloudConnectionStatus::Connecting => write!(f, "Cloud: connecting..."),
            CloudConnectionStatus::Connected => write!(f, "Cloud: connected"),
        }
    }
}

/// Shared, cross-task state - `nodem_task` renders into `runtime` and sets
/// `display_dirty`; `oled_task` reads it back out and clears the flag once
/// it has flushed that frame to the physical display. `uart_task` appends
/// incoming bytes to `command_buf`/`command_buf_pos`; `nodem_task` drains
/// and processes whatever complete commands have accumulated there. Guarded
/// by a `Mutex` rather than ESP32's `Rc<RefCell<_>>` since there's no
/// `alloc::rc` analog worth reaching for here - all three tasks run
/// cooperatively on the same embassy executor, so a `NoopRawMutex` is
/// enough.
pub struct NodemState {
    pub cloud_status: CloudStatus,
    pub runtime: DOM,
    pub display_dirty: bool,
    pub command_buf: [u8; COMMAND_BUF_LEN],
    pub command_buf_pos: usize,
}

impl NodemState {
    pub fn new() -> Self {
        // Boxed (then leaked) so the buffer's address stays fixed even as
        // `NodemState` itself gets moved into the `Mutex`/`StaticCell` below -
        // `Surface` holds a raw pointer into it, same reasoning as the ESP32
        // `Global::display_buffer` field.
        let buffer: Box<[u8]> = Box::new([0u8; BUFFER_LEN]);

        Self {
            cloud_status: CloudStatus::new(),
            runtime: DOM::new(Box::leak(buffer), DISPLAY_WIDTH as _, DISPLAY_HEIGHT as _),
            display_dirty: false,
            command_buf: [0u8; COMMAND_BUF_LEN],
            command_buf_pos: 0,
        }
    }
}

pub type NodemMutex = Mutex<NoopRawMutex, NodemState>;

/// Advances the `nodem_rs` DOM runtime, re-renders the cloud status overlay
/// into the shared framebuffer, and drains any commands `uart_task` has
/// buffered - all 60 times a second. Only touches the in-memory framebuffer
/// (marking it dirty for `oled_task`) and, when a drained command produced a
/// reply, `uart_tx` - never talks to the display itself.
///
/// The overlay is suppressed once the cloud connection has been up for
/// `HIDE_REPORT_AFTER` straight - `connected_since` tracks the start of the
/// current unbroken "connected" streak (reset to `None` the moment it
/// drops), so a hidden report reappears immediately on any disconnect.
#[embassy_executor::task]
pub async fn nodem_task(state: &'static NodemMutex, mut uart_tx: BufferedUarteTx<'static>) {
    let mut connected_since: Option<Instant> = None;

    loop {
        let mut response_buf = [0u8; RESPONSE_BUF_LEN];
        let mut response_len = 0;

        {
            let mut guard = state.lock().await;
            let s = &mut *guard;

            s.runtime.run();

            let connected = s.cloud_status.connection == CloudConnectionStatus::Connected;
            connected_since = match (connected, connected_since) {
                (true, since @ Some(_)) => since,
                (true, None) => Some(Instant::now()),
                (false, _) => None,
            };

            let show_report = connected_since.map_or(true, |since| since.elapsed() < HIDE_REPORT_AFTER);

            if show_report {
                let font = 0;
                let message = format!("{}", s.cloud_status);
                let size = s.runtime.surface.get_text_size(Identifier::Index(font), message.as_str());
                s.runtime.surface.draw_rect(
                    Area { point: Point { x: 0, y: 0 }, size: Size { width: size.width + 6, height: size.height + 6 } },
                    1,
                );
                s.runtime.surface.fill_rect(
                    Area { point: Point { x: 1, y: 1 }, size: Size { width: size.width + 4, height: size.height + 4 } },
                    0,
                );
                s.runtime.surface.draw_text(Identifier::Index(font), message.as_str(), Point { x: 3, y: 3 });
            }

            s.display_dirty = true;

            // Drain every complete command currently buffered - `uart_task`
            // may have appended more than one line's worth since we last
            // looked. `process_command` returns 0 consumed when what's left
            // isn't a complete command yet, which is also our stopping
            // condition for "buffer full of garbage, give up" below.
            let mut control = NoopControl;
            while s.command_buf_pos > 0 {
                let mut writer = BufWriter { buf: &mut response_buf[response_len..], pos: 0 };                
                let (consumed, _) =
                    s.runtime.process_command(&s.command_buf[..s.command_buf_pos], &mut writer, &mut control);
                response_len += writer.pos;                
                if consumed == 0 {
                    break;
                }
                s.command_buf.copy_within(consumed..s.command_buf_pos, 0);
                s.command_buf_pos -= consumed;
            }
            if s.command_buf_pos >= COMMAND_BUF_LEN {
                defmt::warn!("uart command buffer full with no complete command, dropping it");
                s.command_buf_pos = 0;
            }
        }

        // `write` follows embedded-io's "may write fewer bytes than given"
        // contract rather than guaranteeing the whole buffer goes out in one
        // call - loop until it's all been accepted instead of dropping
        // whatever didn't fit in a single call.
        let mut sent = 0;
        while sent < response_len {
            match uart_tx.write(&response_buf[sent..response_len]).await {
                Ok(0) | Err(_) => {
                    defmt::warn!("uart write failed, dropping rest of response");
                    break;
                }
                Ok(n) => sent += n,
            }
        }

        Timer::after_millis(1000 / 60).await;
    }
}
