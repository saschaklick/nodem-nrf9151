use embassy_nrf::buffered_uarte::BufferedUarteRx;

use crate::nodem::{NodemMutex, COMMAND_BUF_LEN};

/// Watches the UART for incoming bytes and appends them to the shared
/// `NodemState::command_buf` - `nodem_task` is the one that actually parses
/// and acts on whatever accumulates there (via `process_command`), so this
/// task only ever reads and buffers.
#[embassy_executor::task]
pub async fn uart_task(mut rx: BufferedUarteRx<'static>, state: &'static NodemMutex) {
    let mut chunk = [0u8; 64];

    loop {
        let n = match rx.read(&mut chunk).await {
            Ok(n) => n,
            Err(e) => {
                defmt::warn!("uart read error: {:?}", e);
                continue;
            }
        };

        let mut s = state.lock().await;
        let free = COMMAND_BUF_LEN - s.command_buf_pos;
        let copy = n.min(free);
        let pos = s.command_buf_pos;
        s.command_buf[pos..pos + copy].copy_from_slice(&chunk[..copy]);
        s.command_buf_pos += copy;
        if copy < n {
            defmt::warn!("uart command buffer full, dropping {} byte(s)", n - copy);
        }
    }
}
