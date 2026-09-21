#ifndef DISPLAY_TASK_H_
#define DISPLAY_TASK_H_

#include <stddef.h>
#include <stdint.h>

#define DISPLAY_WIDTH     128
#define DISPLAY_HEIGHT    64
#define DISPLAY_BUF_SIZE  ((DISPLAY_WIDTH * DISPLAY_HEIGHT) / 8)

/*
 * Copies `len` bytes (must equal DISPLAY_BUF_SIZE) from `buf` into the
 * shared display buffer under lock and marks it dirty for display_task to
 * flush to the panel on its next cycle. `buf` must already be in the
 * SSD1306's native page-tiled GDDRAM format: byte index = (y/8)*width + x,
 * bit = y%8 with the LSB as the top row of the page.
 *
 * Safe to call from any thread; the copy happens under `display_mutex` so a
 * writer never races the display task's own read of the buffer.
 */
void display_task_submit(const uint8_t *buf, size_t len);

#endif /* DISPLAY_TASK_H_ */
