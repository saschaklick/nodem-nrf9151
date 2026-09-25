#ifndef DISPLAY_TASK_H_
#define DISPLAY_TASK_H_

#include <stddef.h>
#include <stdint.h>

#include "nodem_config.h"

/*
 * Copies nodem-rs's framebuffer `buf` (nodem_config_buffer_len(config)
 * bytes, row-major, MSB first - bit index = y * width + x) and the config
 * it was rendered with into the shared display buffer under lock, and
 * marks it dirty for display_task to flush to the panel on its next cycle.
 * display_task maps it onto the configured panel itself, through the
 * config's "oled" entry (see nodem_config.h) and the panel's own
 * size/offset/rotation (see display_oled_set()).
 *
 * Safe to call from any thread; the copy happens under `display_mutex` so a
 * writer never races the display task's own read of the buffer.
 */
void display_task_submit(const uint8_t *buf, const struct nodem_config *config);

/*
 * Validates and persists config_store's "oled" key, then has display_task
 * tear down the current panel and bring it up anew with it right away.
 * Same format as nodem-esp32's oled.rs (OledConfig):
 * "<protocol>:<width>:<height>:<x>:<y>[:<rotation>]", default
 * "ssd1306:128:64:0:0". `ssd1306` is the only protocol; <width>x<height>
 * is the panel's own size (128x64, 128x32, 96x16, 72x40, 64x48 or 64x32),
 * <x> (0-127) the start column in the controller's display RAM, <y> (0-63)
 * its display offset register (a vertical shift, per row - unlike
 * nodem-esp32, not limited to multiples of 8), <rotation> 0/90/180/270
 * clockwise, done by the controller's column/row flips (90/270 also need
 * a software transpose, which the controller can't do). All of it is set
 * on the controller on every (re)initialization. An empty value or any
 * other protocol disables the display (stored as given). A valid
 * "ssd1306" value is stored normalized (rotation only when non-zero).
 *
 * Returns 0 on success, -EINVAL for a malformed "ssd1306" value (nothing
 * stored), or config_store's error if persisting it failed.
 */
int display_oled_set(const char *value);

#endif /* DISPLAY_TASK_H_ */
