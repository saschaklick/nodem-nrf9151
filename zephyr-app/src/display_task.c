/*
 * Hand-rolled SSD1306 driver over plain Zephyr i2c_write() calls, run as its
 * own thread that flushes whatever the latest submitted frame is to the
 * panel. Written after Zephyr's own built-in `ssd1306` display driver
 * produced garbled output on this exact panel for reasons never root-caused
 * - this raw implementation (init sequence and command bytes copied from
 * the proven-working `ssd1306-embassy` driver used by this project's
 * earlier embassy-based firmware) was confirmed correct on real hardware
 * first via a static test pattern before being wired up as an async task.
 *
 * The panel itself is configurable (config_store's "oled", see
 * display_oled_set()) - same format and behaviour as nodem-esp32's
 * oled.rs, whose ssd1306 crate this driver's per-size init differences
 * (multiplex ratio, COM pin config, the 72x40's internal IREF) and column
 * offsets are taken from.
 */

#include "display_task.h"

#include <errno.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>

#include "config_store.h"
#include "log.h"
#include "nodem_config.h"

#define TAG "display"

#define SSD1306_ADDR 0x3C

#define CMD_DISPLAY_OFF         0xAE
#define CMD_DISPLAY_ON          0xAF
#define CMD_SET_CONTRAST        0x81
#define CMD_NORMAL_DISPLAY      0xA6
#define CMD_SET_DISPLAY_CLK_DIV 0xD5
#define CMD_SET_MULTIPLEX       0xA8
#define CMD_SET_DISPLAY_OFFSET  0xD3
#define CMD_SET_START_LINE      0x40
#define CMD_CHARGE_PUMP         0x8D
#define CMD_MEMORY_MODE         0x20
#define CMD_SEG_REMAP_NORMAL    0xA0 /* column 0 -> SEG0 */
#define CMD_SEG_REMAP           0xA1 /* column 127 -> SEG0 */
#define CMD_COM_SCAN_INC        0xC0 /* scan COM0 -> COM[N-1] */
#define CMD_COM_SCAN_DEC        0xC8 /* scan COM[N-1] -> COM0 */
#define CMD_SET_COM_PINS        0xDA
#define CMD_SET_PRECHARGE       0xD9
#define CMD_SET_VCOM_DETECT     0xDB
#define CMD_ENTIRE_DISPLAY_ON   0xA4
#define CMD_DEACTIVATE_SCROLL   0x2E
#define CMD_SET_COLUMN_ADDR     0x21
#define CMD_SET_PAGE_ADDR       0x22
#define CMD_INTERNAL_IREF       0xAD

/* The SSD1306's own display RAM - every panel size is a window into it. */
#define SSD1306_RAM_COLS 128
#define SSD1306_RAM_ROWS 64

#define OLED_KEY         "oled"
#define OLED_DEFAULT     "ssd1306:128:64:0:0"
/* config_store's MAX_STR_LEN (127) + NUL. */
#define OLED_VALUE_LEN   128

struct oled_config {
	uint8_t width;
	uint8_t height;
	uint8_t x;
	uint8_t y;
	uint16_t rotation;
};

/* The panel sizes nodem-esp32's ssd1306 crate supports, with that crate's
 * per-size column offset (DisplaySize::OFFSETX) and init differences. */
struct oled_size {
	uint8_t width;
	uint8_t height;
	uint8_t offset_x;
	uint8_t com_pins;
	bool iref;
};

static const struct oled_size oled_sizes[] = {
	{ 128, 64, 0, 0x12, false },
	{ 128, 32, 0, 0x02, false },
	{ 96, 16, 0, 0x02, false },
	{ 72, 40, 28, 0x12, true },
	{ 64, 48, 32, 0x12, false },
	{ 64, 32, 32, 0x12, false },
};

static const struct oled_size *oled_size_find(uint8_t width, uint8_t height)
{
	for (size_t i = 0; i < ARRAY_SIZE(oled_sizes); i++) {
		if (oled_sizes[i].width == width && oled_sizes[i].height == height) {
			return &oled_sizes[i];
		}
	}

	return NULL;
}

/* Parses the next ':'-separated field of `*s` as a decimal number (spaces
 * around it allowed), advancing `*s` past it and its separator. False if
 * there's no field left or it isn't a plain number. */
static bool next_num_field(const char **s, long *out)
{
	if (*s == NULL) {
		return false;
	}

	char *end;
	long v = strtol(*s, &end, 10);

	if (end == *s) {
		return false;
	}
	while (*end == ' ') {
		end++;
	}
	if (*end == ':') {
		*s = end + 1;
	} else if (*end == '\0') {
		*s = NULL;
	} else {
		return false;
	}

	*out = v;
	return true;
}

/* 1 (enabled, `out` filled in), 0 (disabled: empty, or a protocol other
 * than "ssd1306") or -EINVAL (a malformed "ssd1306" value) - same rules as
 * nodem-esp32's OledConfig::parse(). */
static int oled_config_parse(const char *s, struct oled_config *out)
{
	while (*s == ' ') {
		s++;
	}

	const char *proto_end = strchr(s, ':');
	size_t proto_len = proto_end ? (size_t)(proto_end - s) : strlen(s);

	while (proto_len > 0 && s[proto_len - 1] == ' ') {
		proto_len--;
	}
	if (proto_len != strlen("ssd1306") || strncmp(s, "ssd1306", proto_len) != 0) {
		return 0;
	}

	const char *p = proto_end ? proto_end + 1 : NULL;
	long width, height, x, y, rotation = 0;

	if (!next_num_field(&p, &width) || !next_num_field(&p, &height) ||
	    !next_num_field(&p, &x) || !next_num_field(&p, &y)) {
		return -EINVAL;
	}
	if (p != NULL && (!next_num_field(&p, &rotation) || p != NULL)) {
		return -EINVAL;
	}

	if (width < 0 || width > 255 || height < 0 || height > 255 ||
	    !oled_size_find((uint8_t)width, (uint8_t)height) ||
	    x < 0 || x >= SSD1306_RAM_COLS || y < 0 || y >= SSD1306_RAM_ROWS ||
	    (rotation != 0 && rotation != 90 && rotation != 180 && rotation != 270)) {
		return -EINVAL;
	}

	*out = (struct oled_config){
		.width = (uint8_t)width,
		.height = (uint8_t)height,
		.x = (uint8_t)x,
		.y = (uint8_t)y,
		.rotation = (uint16_t)rotation,
	};
	return 1;
}

/* Rotation only when non-zero, so the default round-trips to exactly
 * OLED_DEFAULT - same as nodem-esp32's OledConfig::to_nvs_string(). */
static void oled_config_format(const struct oled_config *c, char *buf, size_t buf_len)
{
	int n = snprintf(buf, buf_len, "ssd1306:%u:%u:%u:%u", c->width, c->height, c->x, c->y);

	if (c->rotation != 0 && n > 0 && (size_t)n < buf_len) {
		snprintf(buf + n, buf_len - n, ":%u", c->rotation);
	}
}

/* Bumped by display_oled_set() - display_task() reinitializes the panel
 * whenever it no longer matches the generation it last set up. */
static atomic_t oled_generation = ATOMIC_INIT(0);

int display_oled_set(const char *value)
{
	struct oled_config config;
	char normalized[OLED_VALUE_LEN];
	int parsed = oled_config_parse(value, &config);

	if (parsed < 0) {
		return parsed;
	}
	if (parsed > 0) {
		oled_config_format(&config, normalized, sizeof(normalized));
		value = normalized;
	}

	int err = config_set_str(OLED_KEY, value);

	if (!err) {
		atomic_inc(&oled_generation);
	}

	return err;
}

/* Reads "oled" back, the same way nodem-esp32's read_oled_config() does: a
 * missing value (first boot) or a malformed "ssd1306" one is replaced in
 * config_store by OLED_DEFAULT (so "#cfg" shows what's actually in use); a
 * deliberately disabled value is kept as it is. False if disabled. */
static bool oled_config_read(struct oled_config *out)
{
	/* Distinguishes "never set" from a deliberately stored "" (disabled),
	 * which config_get_str()'s default alone can't. */
	static const char unset[] = "\x01";
	char value[OLED_VALUE_LEN];

	config_get_str(OLED_KEY, value, sizeof(value), unset);

	if (strcmp(value, unset) != 0) {
		int parsed = oled_config_parse(value, out);

		if (parsed >= 0) {
			return parsed > 0;
		}
		warn(TAG, "malformed '%s' in config ('%s'), falling back to default", OLED_KEY,
		     value);
	}

	if (config_set_str(OLED_KEY, OLED_DEFAULT)) {
		error(TAG, "failed to persist default '%s'", OLED_KEY);
	}

	return oled_config_parse(OLED_DEFAULT, out) > 0;
}

static int write_cmd(const struct device *i2c, uint8_t cmd)
{
	uint8_t buf[2] = { 0x00, cmd };

	return i2c_write(i2c, buf, sizeof(buf), SSD1306_ADDR);
}

/* How each rotation is done on the controller itself, relative to
 * rotation 0's (this panel's proven default) column-127-to-SEG0 remap and
 * reversed COM scan: 180 flips both, 90/270 each flip one. The SSD1306
 * has no way to swap rows and columns, so 90/270 are the one thing left
 * in software - a plain transpose (see panel_pixel()); the flip that
 * turns that transpose into a rotation is again the controller's. */
static bool rotation_flips_cols(uint16_t rotation)
{
	return rotation == 90 || rotation == 180;
}

static bool rotation_flips_rows(uint16_t rotation)
{
	return rotation == 180 || rotation == 270;
}

static bool rotation_transposes(uint16_t rotation)
{
	return rotation == 90 || rotation == 270;
}

static int ssd1306_init(const struct device *i2c, const struct oled_size *size,
			const struct oled_config *config)
{
	int err = 0;

	err |= write_cmd(i2c, CMD_DISPLAY_OFF);

	err |= write_cmd(i2c, CMD_SET_DISPLAY_CLK_DIV);
	err |= write_cmd(i2c, 0x80);

	err |= write_cmd(i2c, CMD_SET_MULTIPLEX);
	err |= write_cmd(i2c, size->height - 1); /* 1/<height> duty */

	/* The configured y offset: a vertical shift by COM, per row. */
	err |= write_cmd(i2c, CMD_SET_DISPLAY_OFFSET);
	err |= write_cmd(i2c, config->y);

	err |= write_cmd(i2c, CMD_SET_START_LINE);

	err |= write_cmd(i2c, CMD_CHARGE_PUMP);
	err |= write_cmd(i2c, 0x14); /* enable charge pump */

	err |= write_cmd(i2c, CMD_MEMORY_MODE);
	err |= write_cmd(i2c, 0x00); /* horizontal addressing mode */

	err |= write_cmd(i2c, rotation_flips_cols(config->rotation) ? CMD_SEG_REMAP_NORMAL
								   : CMD_SEG_REMAP);
	err |= write_cmd(i2c, rotation_flips_rows(config->rotation) ? CMD_COM_SCAN_INC
								   : CMD_COM_SCAN_DEC);

	err |= write_cmd(i2c, CMD_SET_COM_PINS);
	err |= write_cmd(i2c, size->com_pins);

	if (size->iref) {
		err |= write_cmd(i2c, CMD_INTERNAL_IREF);
		err |= write_cmd(i2c, 0x30); /* internal IREF, 240uA */
	}

	err |= write_cmd(i2c, CMD_SET_CONTRAST);
	err |= write_cmd(i2c, 0xCF);

	err |= write_cmd(i2c, CMD_SET_PRECHARGE);
	err |= write_cmd(i2c, 0xF1);

	err |= write_cmd(i2c, CMD_SET_VCOM_DETECT);
	err |= write_cmd(i2c, 0x40);

	err |= write_cmd(i2c, CMD_ENTIRE_DISPLAY_ON);
	err |= write_cmd(i2c, CMD_NORMAL_DISPLAY);
	err |= write_cmd(i2c, CMD_DEACTIVATE_SCROLL);

	err |= write_cmd(i2c, CMD_DISPLAY_ON);

	return err;
}

/* Sets the display RAM window the following ssd1306_write_page() calls
 * fill: columns `col`..`col`+`cols`-1, pages `page`..`page`+`pages`-1. In
 * horizontal addressing mode (as ssd1306_init() sets up) the controller's
 * write pointer advances through it column by column and wraps to the next
 * page on its own, and it keeps its position between I2C transfers. All
 * six command bytes go out as one transfer (control byte 0x00 = a stream
 * of commands follows) rather than one per byte - this is sent once per
 * changed page, so its overhead counts. */
static int ssd1306_set_window(const struct device *i2c, uint8_t col, uint8_t cols, uint8_t page,
			      uint8_t pages)
{
	uint8_t buf[] = {
		0x00,
		CMD_SET_COLUMN_ADDR, col, col + cols - 1,
		CMD_SET_PAGE_ADDR, page, page + pages - 1,
	};

	return i2c_write(i2c, buf, sizeof(buf), SSD1306_ADDR);
}

/* One page (8 rows) of the frame - one byte per column, LSB at the top -
 * built into page_buf[1..], then (whatever of it changed) sent, then the
 * next page is built into the same buffer. Only ever one page in RAM,
 * rather than the whole frame. page_buf[0] is room for the I2C data
 * control byte in front of column 0. */
static uint8_t page_buf[1 + SSD1306_RAM_COLS];

/* What each page's columns were last sent as - page_buf is compared
 * against this so only what actually changed goes out over I2C (see
 * display_task()). Indexed like page_buf's data: [page][column in the
 * draw area]. Only trusted while `sent_valid` - after a failed write or a
 * reinit the panel's RAM contents are unknown, so the next update resends
 * everything. */
static uint8_t sent[SSD1306_RAM_ROWS / 8][SSD1306_RAM_COLS];
static bool sent_valid;

/* Sends columns `first`..`first`+`len`-1 of page_buf's data as a single
 * transfer - the data control byte (0x40) goes into the byte right before
 * them, page_buf[first] (the previous column's data, or page_buf[0]),
 * which is saved and restored around the transfer. */
static int ssd1306_write_page(const struct device *i2c, uint8_t first, uint8_t len)
{
	uint8_t saved = page_buf[first];

	page_buf[first] = 0x40;
	int err = i2c_write(i2c, &page_buf[first], 1 + len, SSD1306_ADDR);

	page_buf[first] = saved;
	return err;
}

/* Zeroes all of display RAM, so nothing a previous config left outside the
 * new draw area can show through - which also makes `sent` (all zeros)
 * exactly what the panel holds. */
static int ssd1306_clear(const struct device *i2c)
{
	int err = ssd1306_set_window(i2c, 0, SSD1306_RAM_COLS, 0, SSD1306_RAM_ROWS / 8);

	memset(&page_buf[1], 0, SSD1306_RAM_COLS);
	for (int pg = 0; pg < SSD1306_RAM_ROWS / 8 && !err; pg++) {
		err = ssd1306_write_page(i2c, 0, SSD1306_RAM_COLS);
	}

	memset(sent, 0, sizeof(sent));
	sent_valid = err == 0;

	return err;
}

/* Which framebuffer coordinate driver pixel `d` (of `size` along this axis)
 * reads - nodem-esp32's DriverView::pixel() axis mapping: a framebuffer
 * smaller than the driver's output once scaled is centered on it, then
 * everything is shifted by the mapping's offset. */
static int16_t map_axis(int d, int size, int fb_size, int scale, int shift)
{
	int margin = MAX((size - fb_size * scale) / 2, 0);
	int rel = d - margin;
	/* Floor division, so pixels left of/above the margin land on
	 * negative (off) framebuffer coordinates rather than rounding onto
	 * the first one. */
	int q = rel >= 0 ? rel / scale : -((-rel + scale - 1) / scale);

	return (int16_t)CLAMP(shift + q, INT16_MIN, INT16_MAX);
}

/* nodem_task.c's own framebuffer (no copy of it here) and the config it's
 * rendered with - all only ever touched under display_mutex. nodem_task
 * holds it for each whole render (display_fb_lock()), display_task() only
 * while building one page at a time, releasing it for each page's I2C
 * transfer - so every page comes from a completely rendered frame and
 * rendering never waits on the bus, but consecutive pages of one panel
 * update can come from consecutive frames. */
static const uint8_t *display_fb;
static struct nodem_config display_nodem;
K_MUTEX_DEFINE(display_mutex);
static atomic_t display_dirty = ATOMIC_INIT(0);

void display_fb_lock(void)
{
	k_mutex_lock(&display_mutex, K_FOREVER);
}

void display_fb_unlock(void)
{
	k_mutex_unlock(&display_mutex);
}

void display_task_submit(const uint8_t *buf, const struct nodem_config *config)
{
	k_mutex_lock(&display_mutex, K_FOREVER);
	display_fb = buf;
	display_nodem = *config;
	k_mutex_unlock(&display_mutex);

	atomic_set(&display_dirty, 1);
}

/* Everything build_page() maps through, rebuilt by frame_prepare() only
 * when the panel config or display_nodem actually changed. */
static struct {
	bool valid;
	/* What the tables below were built from. */
	struct oled_config oc;
	uint16_t fb_width;
	uint16_t fb_height;
	struct nodem_mapping map;

	bool transposed;
	/* The driver's output size as nodem sees it - swapped when
	 * transposed. */
	int view_w;
	int view_h;
	/* Per panel column/row, the bit offset it contributes to a
	 * framebuffer pixel's bit index (y * width + x): x for columns and
	 * y * width for rows - or the other way round when transposed - so
	 * a pixel's bit index is just col_bit[px] + row_bit[py]. -1 where
	 * the column/row falls outside the framebuffer (reads as off),
	 * which settles bounds and rotation up front, once, instead of per
	 * pixel. */
	int32_t col_bit[SSD1306_RAM_COLS];
	int32_t row_bit[SSD1306_RAM_ROWS];
} fm;

/* The bit offset framebuffer coordinate `v` contributes along an axis of
 * `size` pixels, each `stride` bits apart - or -1 outside it. */
static int32_t axis_bit(int v, int size, int stride)
{
	return v >= 0 && v < size ? (int32_t)v * stride : -1;
}

/* Call with display_mutex held - checked for every page, since the config
 * can change between two pages of one panel update. */
static void frame_prepare(const struct oled_config *oc)
{
	const struct nodem_config *nc = &display_nodem;
	const struct nodem_mapping *m = &nc->oled;

	if (fm.valid && fm.oc.width == oc->width && fm.oc.height == oc->height &&
	    fm.oc.rotation == oc->rotation && fm.fb_width == nc->width &&
	    fm.fb_height == nc->height && fm.map.present == m->present && fm.map.x == m->x &&
	    fm.map.y == m->y && fm.map.scale_x == m->scale_x && fm.map.scale_y == m->scale_y) {
		return;
	}

	fm.valid = true;
	fm.oc = *oc;
	fm.fb_width = nc->width;
	fm.fb_height = nc->height;
	fm.map = *m;

	fm.transposed = rotation_transposes(oc->rotation);
	fm.view_w = fm.transposed ? oc->height : oc->width;
	fm.view_h = fm.transposed ? oc->width : oc->height;

	if (!m->present) {
		return;
	}

	int w = nc->width;
	int h = nc->height;

	for (int c = 0; c < oc->width; c++) {
		fm.col_bit[c] =
			fm.transposed
				? axis_bit(map_axis(c, fm.view_h, h, m->scale_y, m->y), h, w)
				: axis_bit(map_axis(c, fm.view_w, w, m->scale_x, m->x), w, 1);
	}
	for (int r = 0; r < oc->height; r++) {
		fm.row_bit[r] =
			fm.transposed
				? axis_bit(map_axis(r, fm.view_w, w, m->scale_x, m->x), w, 1)
				: axis_bit(map_axis(r, fm.view_h, h, m->scale_y, m->y), h, w);
	}
}

/* nodem-esp32's DriverView::fallback_pixel() for a driver without a
 * mapping: every other pixel around the edge, which ones alternating once
 * a second. */
static void build_fallback_page(uint8_t skip, uint8_t cols, int pg)
{
	int phase = (int)((k_uptime_get() / MSEC_PER_SEC) % 2);

	for (int c = 0; c < cols; c++) {
		int px = skip + c;
		uint8_t byte = 0;

		for (int b = 0; b < 8; b++) {
			int py = pg * 8 + b;
			int vx = fm.transposed ? py : px;
			int vy = fm.transposed ? px : py;
			bool edge = vx == 0 || vy == 0 || vx == fm.view_w - 1 || vy == fm.view_h - 1;

			if (edge && (vx + vy + phase) % 2 == 0) {
				byte |= BIT(b);
			}
		}
		page_buf[1 + c] = byte;
	}
}

/* Builds page `pg` of the frame for panel columns skip..skip+cols-1 into
 * page_buf, through the nodem config's "oled" mapping (or the fallback
 * frame, without one). Rows and columns of the panel swap roles for a
 * transposed (90/270) rotation - the one part of it the controller can't
 * do itself - which frame_prepare()'s tables already account for. Call
 * with display_mutex held, after frame_prepare(). */
static void build_page(uint8_t skip, uint8_t cols, int pg)
{
	if (!display_nodem.oled.present) {
		build_fallback_page(skip, cols, pg);
		return;
	}

	/* This page's 8 rows, loaded once for all its columns; the ones
	 * outside the framebuffer are dropped here rather than checked per
	 * pixel. */
	int32_t rows[8];
	uint8_t row_mask[8];
	int n = 0;

	for (int b = 0; b < 8; b++) {
		int32_t r = fm.row_bit[pg * 8 + b];

		if (r >= 0) {
			rows[n] = r;
			row_mask[n] = BIT(b);
			n++;
		}
	}

	if (n == 0) {
		memset(&page_buf[1], 0, cols);
		return;
	}

	const uint8_t *fb = display_fb;

	for (int c = 0; c < cols; c++) {
		int32_t col = fm.col_bit[skip + c];
		uint8_t byte = 0;

		if (col >= 0) {
			for (int i = 0; i < n; i++) {
				uint32_t bit = (uint32_t)(col + rows[i]);

				if (fb[bit >> 3] & (0x80 >> (bit & 7))) {
					byte |= row_mask[i];
				}
			}
		}
		page_buf[1 + c] = byte;
	}
}

#define DISPLAY_STACK_SIZE 1024
#define DISPLAY_PRIORITY   5
#define DISPLAY_POLL_MS    20

static void display_task(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	const struct device *i2c = DEVICE_DT_GET(DT_NODELABEL(i2c3));

	if (!device_is_ready(i2c)) {
		error(TAG, "i2c device not ready");
		return;
	}


	/* Anything but the current generation, so the first pass sets up. */
	atomic_val_t generation = atomic_get(&oled_generation) - 1;
	struct oled_config config;
	bool initialized = false;
	/* The write window in display RAM, and how many of the frame's own
	 * leading columns fall before it (cut off at the RAM's edge). */
	uint8_t col = 0, cols = 0, skip = 0, pages = 0;

	while (true) {
		if (atomic_get(&oled_generation) != generation) {
			generation = atomic_get(&oled_generation);

			/* Blank the panel rather than leave it frozen on the
			 * last frame - the new config may be a different
			 * size/offset, or disabled altogether. */
			if (initialized && write_cmd(i2c, CMD_DISPLAY_OFF)) {
				warn(TAG, "ssd1306 switch-off failed");
			}
			initialized = false;
			sent_valid = false;

			if (oled_config_read(&config)) {
				const struct oled_size *size =
					oled_size_find(config.width, config.height);
				int err = ssd1306_init(i2c, size, &config);

				info(TAG, "ssd1306_init %ux%u+%u+%u rot %u err=%d", config.width,
				     config.height, config.x, config.y, config.rotation, err);
				initialized = err == 0;

				/* The draw area's start column in display RAM:
				 * the size's own column offset plus the
				 * configured x - mirrored when the controller
				 * flips columns, so the image stays on the
				 * same glass. There's no horizontal
				 * counterpart to the display offset register,
				 * so x is the column address itself. y is
				 * the display offset register (see
				 * ssd1306_init()), so the frame always starts
				 * at page 0. */
				int start = size->offset_x + config.x;

				if (rotation_flips_cols(config.rotation)) {
					start = SSD1306_RAM_COLS - start - config.width;
				}
				skip = start < 0 ? -start : 0;
				col = start + skip;
				cols = MAX(MIN(config.width, SSD1306_RAM_COLS - start) - skip, 0);
				pages = config.height / 8;

				if (initialized && ssd1306_clear(i2c)) {
					warn(TAG, "ssd1306 clear failed");
				}

				/* First frame goes out right away - the panel
				 * has just been (re)initialized and shows
				 * nothing yet. */
				atomic_set(&display_dirty, 1);
			} else {
				info(TAG, "oled disabled");
			}
		}

		if (initialized && atomic_cas(&display_dirty, 1, 0)) {
			int err = 0;

			/* Every page is built, but only sent if it differs
			 * from what the panel already shows (`sent`) - and
			 * then only from its first to its last changed
			 * column. A static screen costs no I2C traffic at
			 * all. */
			for (int pg = 0; pg < pages && cols > 0 && !err; pg++) {
				k_mutex_lock(&display_mutex, K_FOREVER);
				if (display_fb) {
					frame_prepare(&config);
					build_page(skip, cols, pg);
				} else {
					memset(&page_buf[1], 0, cols);
				}
				k_mutex_unlock(&display_mutex);

				const uint8_t *data = &page_buf[1];
				int first = 0;
				int last = cols - 1;

				if (sent_valid) {
					while (first < cols && data[first] == sent[pg][first]) {
						first++;
					}
					if (first == cols) {
						continue;
					}
					while (data[last] == sent[pg][last]) {
						last--;
					}
				}

				uint8_t len = last - first + 1;

				err = ssd1306_set_window(i2c, col + first, len, pg, 1);
				if (!err) {
					err = ssd1306_write_page(i2c, first, len);
				}
				if (!err) {
					memcpy(&sent[pg][first], &data[first], len);
				}
			}

			/* Only once every page has gone out - a frame that
			 * stopped partway (err) leaves it false, so the next
			 * one resends everything. */
			if (!err && cols > 0) {
				sent_valid = true;
			}

			/* Same as nodem-esp32's run_ssd1306(): a failed
			 * initial init leaves the panel alone until the
			 * config changes; a failed flush reinitializes, and
			 * the next frame tries again. */
			if (err) {
				sent_valid = false;
				err = ssd1306_init(
					i2c, oled_size_find(config.width, config.height), &config);
				error(TAG, "ssd1306 write failed, reinitialized err=%d", err);
			}
		}
		k_msleep(DISPLAY_POLL_MS);
	}
}

K_THREAD_DEFINE(display_tid, DISPLAY_STACK_SIZE, display_task, NULL, NULL, NULL,
		 DISPLAY_PRIORITY, 0, 0);
