#include "nodem_task.h"

#include <string.h>
#include <stdbool.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include "display_task.h"
#include "modem_task.h"
#include "nodem_ffi.h"
#include "nodem_listener.h"
#include "uart_task.h"
#include "log.h"

#define TAG "nodem"

#define NODEM_WIDTH    DISPLAY_WIDTH
#define NODEM_HEIGHT   DISPLAY_HEIGHT
#define NODEM_FB_SIZE  DISPLAY_BUF_SIZE

static uint8_t nodem_fb[NODEM_FB_SIZE];
static uint8_t vtiled_fb[NODEM_FB_SIZE];

/* nodem-rs packs pixels row-major, MSB first: bit_index = y * width + x. The
 * SSD1306 GDDRAM wants them page-tiled instead: byte (y/8)*width + x, bit
 * (y%8) LSB-first (LSB = top row of the page). */
static void convert_to_vtiled(const uint8_t *nodem_buf, uint8_t *out)
{
	memset(out, 0, NODEM_FB_SIZE);

	for (int y = 0; y < NODEM_HEIGHT; y++) {
		for (int x = 0; x < NODEM_WIDTH; x++) {
			int bit_idx = y * NODEM_WIDTH + x;
			bool on = nodem_buf[bit_idx / 8] & (1 << (7 - (bit_idx % 8)));

			if (on) {
				out[(y / 8) * NODEM_WIDTH + x] |= BIT(y % 8);
			}
		}
	}
}

/* Level comes from nodem-rs's `log` crate at runtime (INFO/WARN/ERROR/...),
 * so this can't go through the fixed-level info()/warn()/error() macros -
 * it builds the same "[TAG] LEVEL message" shape directly instead. */
void nodem_log_write(const uint8_t *level_ptr, size_t level_len, const uint8_t *msg_ptr, size_t msg_len)
{
	printk("[%s] %.*s %.*s\n", TAG, (int)level_len, level_ptr, (int)msg_len, msg_ptr);
}

#define NODEM_STACK_SIZE 4096
#define NODEM_PRIORITY   5
#define NODEM_PERIOD_MS  17 /* ~60 Hz */

/* Accumulates bytes pulled from uart_task's ring buffer across cycles.
 * nodem_process_command's underlying line parser only consumes as many
 * bytes as form complete lines - a trailing partial line (no \r/\n yet)
 * reports 0 (or partial) bytes consumed, so any leftover tail has to be
 * kept and retried once more bytes arrive, rather than being fed in fresh
 * (and dropped) every cycle. */
#define CMD_BUF_SIZE 1024
static uint8_t cmd_buf[CMD_BUF_SIZE];
static size_t cmd_buf_len;
static uint8_t cmd_out[CMD_BUF_SIZE];

static void nodem_process_uart(void *runtime)
{
	size_t avail = uart_task_read(&cmd_buf[cmd_buf_len], CMD_BUF_SIZE - cmd_buf_len);

	log_bytes(TAG, "rx", &cmd_buf[cmd_buf_len], avail);

	cmd_buf_len += avail;

	if (cmd_buf_len == 0) {
		return;
	}

	size_t out_len = 0;
	size_t consumed = nodem_process_command(runtime, cmd_buf, cmd_buf_len, cmd_out,
						 sizeof(cmd_out), &out_len);

	if (out_len > 0) {
		log_bytes(TAG, "tx", cmd_out, out_len);
		uart_task_write(cmd_out, out_len);
	}

	if (consumed > 0) {
		memmove(cmd_buf, &cmd_buf[consumed], cmd_buf_len - consumed);
		cmd_buf_len -= consumed;
	} else if (cmd_buf_len == CMD_BUF_SIZE) {
		/* Filled up without ever finding a line terminator (e.g. input
		 * that isn't actually this line-based text protocol, or a
		 * single line longer than CMD_BUF_SIZE) - nodem_process_command
		 * will keep reporting 0 consumed forever otherwise, permanently
		 * wedging this buffer and, transitively, the RX ring buffer
		 * behind it. Drop it and start over rather than stall forever. */
		warn(TAG, "cmd buffer full with no line terminator - dropping %u bytes",
		     (unsigned)cmd_buf_len);
		cmd_buf_len = 0;
	}
}

/* Same shape as nodem_process_uart() (see its own comment on why a partial
 * line has to be carried across calls) but over modem_task.c's websocket
 * bridge (modem_ws_read()/modem_ws_write()) instead of uart_task.c's UART -
 * a wholly separate buffer set, deliberately: merging both transports'
 * bytes into one buffer would lose which transport a given command (and
 * thus its reply) belongs to the moment two commands from different
 * sources interleave.
 *
 * Sized to match modem_task.c's ws_rx_ring (MODEM_WS_RING_SIZE), not tied
 * to CMD_BUF_SIZE - a large "pkg" transfer needs this able to drain that
 * whole ring in one modem_ws_read() call; UART's own CMD_BUF_SIZE has no
 * reason to grow just because this does. */
#define WS_CMD_BUF_SIZE 4096
static uint8_t ws_cmd_buf[WS_CMD_BUF_SIZE];
static size_t ws_cmd_buf_len;
static uint8_t ws_cmd_out[WS_CMD_BUF_SIZE];

static void nodem_process_ws(void *runtime)
{
	size_t avail = modem_ws_read(&ws_cmd_buf[ws_cmd_buf_len], WS_CMD_BUF_SIZE - ws_cmd_buf_len);

	log_bytes(TAG, "ws rx", &ws_cmd_buf[ws_cmd_buf_len], avail);

	ws_cmd_buf_len += avail;

	if (ws_cmd_buf_len == 0) {
		return;
	}

	size_t out_len = 0;
	size_t consumed = nodem_process_command(runtime, ws_cmd_buf, ws_cmd_buf_len, ws_cmd_out,
						 sizeof(ws_cmd_out), &out_len);

	if (out_len > 0) {
		log_bytes(TAG, "ws tx", ws_cmd_out, out_len);
		modem_ws_write(ws_cmd_out, out_len);
	}

	if (consumed > 0) {
		memmove(ws_cmd_buf, &ws_cmd_buf[consumed], ws_cmd_buf_len - consumed);
		ws_cmd_buf_len -= consumed;
	} else if (ws_cmd_buf_len == WS_CMD_BUF_SIZE) {
		/* See nodem_process_uart()'s identical case. */
		warn(TAG, "ws cmd buffer full with no line terminator - dropping %u bytes",
		     (unsigned)ws_cmd_buf_len);
		ws_cmd_buf_len = 0;
	}
}

/* Given by uart_isr() (uart_task.c) and websocket_supervise()
 * (modem_task.c) whenever they queue bytes nodem_task might otherwise not
 * see until its next render deadline - see nodem_task_notify_rx(). Binary
 * (count 1): nodem_task always drains everything currently available on
 * each wake, so a wake that finds nothing new (e.g. two notifies coalesced
 * into one wake) is harmless, and there's nothing to "count". */
K_SEM_DEFINE(nodem_rx_sem, 0, 1);

void nodem_task_notify_rx(void)
{
	k_sem_give(&nodem_rx_sem);
}

static void nodem_task(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	void *runtime = nodem_runtime_new(nodem_fb, sizeof(nodem_fb), NODEM_WIDTH, NODEM_HEIGHT);

	if (!runtime) {
		error(TAG, "nodem_runtime_new failed");
		return;
	}

	/* Deferred until after the first nodem_runtime_run() below, not called
	 * here - DOM::run()'s very first call is nodem's own initialization
	 * (it loads nodem-rs's built-in PKG_SYS as source 0), and a persisted
	 * package loading before that has run is loading into a DOM that
	 * hasn't finished setting itself up yet. */
	bool pkg_loaded = false;

	while (true) {
		int64_t deadline = k_uptime_get() + NODEM_PERIOD_MS;

		/* Drains and processes UART/websocket traffic in a tight
		 * loop until the next render is due, rather than just once
		 * per render frame - a reply (e.g. a "pkg" upload's final
		 * ack) used to wait out however much of NODEM_PERIOD_MS was
		 * left before this loop came back around to send it. Each
		 * pass either finds new bytes and handles them immediately,
		 * or - the common case - blocks on nodem_rx_sem, which wakes
		 * it the instant more arrive; the deadline is just the
		 * upper bound on that wait, so render cadence is unchanged. */
		for (;;) {
			nodem_process_uart(runtime);
			nodem_process_ws(runtime);

			/* Checked after every pass, not just once per render
			 * frame - see nodem_process_uart()'s doc comment on
			 * why this can't fire before the response that
			 * triggered it has actually been sent; that's just as
			 * true here, it just now happens far more often. */
			if (nodem_control_take_restart()) {
				nodem_hw_reset();
			}

			/* A "pkg" upload finished this pass - control.rs's
			 * IControlLoader impl can't load it itself (no access
			 * to the live Media/Surface from there), so it just
			 * flags this; re-running nodem_load_pkg() here applies
			 * it right away instead of leaving it to take effect
			 * only on the next reboot. */
			if (nodem_control_take_pkg_ready()) {
				nodem_load_pkg(runtime);
			}

			int64_t remaining = deadline - k_uptime_get();

			if (remaining <= 0) {
				break;
			}

			k_sem_take(&nodem_rx_sem, K_MSEC(remaining));
		}

		nodem_runtime_run(runtime);

		if (!pkg_loaded) {
			pkg_loaded = true;
			nodem_load_pkg(runtime);
		}

		convert_to_vtiled(nodem_fb, vtiled_fb);
		display_task_submit(vtiled_fb, sizeof(vtiled_fb));
	}
}

K_THREAD_DEFINE(nodem_tid, NODEM_STACK_SIZE, nodem_task, NULL, NULL, NULL, NODEM_PRIORITY, 0, 0);
