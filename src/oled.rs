use embassy_nrf::twim::Twim;
use embassy_time::Timer;
use ssd1306_embassy::Ssd1306;

use crate::nodem::NodemMutex;

/// Flushes the shared `nodem`-rendered framebuffer to the physical SSD1306
/// over I2C whenever `nodem_task` marks it dirty - the split mirrors the
/// ESP32 original's separate `oled_task`, translated from its raw
/// `set_row`/`set_column`/`draw` page writes to `ssd1306-embassy`'s
/// per-pixel `set_pixel`/`flush` API.
#[embassy_executor::task]
pub async fn oled_task(mut display: Ssd1306<Twim<'static>>, state: &'static NodemMutex) {
    if display.init().await.is_err() {
        defmt::error!("ssd1306 init failed");
        return;
    }

    loop {
        {
            let mut s = state.lock().await;

            if !s.display_dirty {
                drop(s);
                // Poll frequently rather than busy-spin: with no `.await` at
                // all in this branch, `nodem_task` (same executor) would
                // never get scheduled.
                Timer::after_millis(1).await;
                continue;
            }

            let width = s.runtime.surface.width as usize;
            let height = s.runtime.surface.height as usize;
            // Safety: `surface.framebuffer` was carved out of a `Box::leak`'d
            // allocation in `NodemState::new` and never freed or resized, so
            // it's always valid for the surface's lifetime.
            let framebuffer: &[u8] = unsafe { &*s.runtime.surface.framebuffer };

            for y in 0..height {
                for x in 0..width {
                    let bit_idx = y * width + x;
                    let on = framebuffer[bit_idx / 8] & (1 << (7 - (bit_idx % 8))) != 0;
                    display.set_pixel(x as u8, y as u8, on);
                }
            }

            s.display_dirty = false;
        }

        if display.flush().await.is_err() {
            defmt::error!("ssd1306 flush failed");
        }
    }
}
