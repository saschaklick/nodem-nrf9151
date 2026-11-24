#ifndef NODEM_CONFIG_H_
#define NODEM_CONFIG_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * Shape of the nodem-rs framebuffer and which part of it each output driver
 * shows, persisted as config_store's "nodem" in the form
 * "<width>:<height>:<bits_per_pixel>[,<device>...]", e.g. the default
 * "128:64:1,oled:0:0:1:1" - same format and rules as nodem-esp32's
 * nodem.rs (NodemConfig) and driver.rs (DeviceMapping), with this
 * firmware's own limits: "oled" and "hub75" are the drivers (each mapped at
 * most once), the framebuffer must fit NODEM_FB_MAX_SIZE, and scales are
 * 1-16.
 *
 * Each <device> is "<driver>:<x>:<y>:<scale_x>:<scale_y>": <x>/<y> shift
 * which part of the framebuffer shows on the driver, in framebuffer pixels
 * (negative is fine; anything outside the framebuffer reads as off), and
 * <scale_x>/<scale_y> is how many driver pixels one framebuffer pixel
 * covers. A (scaled) framebuffer smaller than the driver's output is
 * centered on it along that axis. A driver without an entry gets nothing
 * from nodem and shows a blinking dotted frame instead.
 *
 * nodem_task.c sizes the framebuffer from it at boot and resizes it live
 * whenever nodem_config_set() changes it; display_task.c maps it onto the
 * OLED through the "oled" entry, hub75_task.c onto the LED matrix through
 * the "hub75" one - both through nodem_map_axis().
 */

#include <zephyr/sys/util.h>

/* The pixel format nodem-rs was built with - its `nodem_mode` rustc cfg
 * (nodem-ffi/.cargo/config.toml's rustflags), which CMakeLists.txt reads
 * from there and passes on as NODEM_MODE_<MODE>=1, so both sides always
 * agree. No mode means 1 bit/pixel indexed, the original format. Each mode
 * packs a pixel the way nodem-rs's Color::from_rgb()/Surface::draw_pixel
 * does - see nodem_fb_rgb(). */
#if defined(NODEM_MODE_RGB332)
#define NODEM_BPP 8
#elif defined(NODEM_MODE_RGB444)
#define NODEM_BPP 12
#elif defined(NODEM_MODE_RGB555) || defined(NODEM_MODE_RGB565)
#define NODEM_BPP 16
#elif defined(NODEM_MODE_RGB666) || defined(NODEM_MODE_RGB24)
#define NODEM_BPP 24
#else
#define NODEM_BPP 1
#endif

#define NODEM_COLOR (NODEM_BPP > 1)

/* Bounded by RAM: nodem_task.c's framebuffer is this size (display_task.c
 * and hub75_task.c read it in place). 1 bit/pixel: e.g. 256x128 or 512x64;
 * colour: 128x64 at 24 bits/pixel. */
#if NODEM_COLOR
#define NODEM_FB_MAX_SIZE 24576
#else
#define NODEM_FB_MAX_SIZE 4096
#endif

struct nodem_mapping {
	bool present;
	int16_t x;
	int16_t y;
	uint8_t scale_x;
	uint8_t scale_y;
};

struct nodem_config {
	uint16_t width;
	uint16_t height;
	uint8_t bits_per_pixel;
	struct nodem_mapping oled;
	struct nodem_mapping hub75;
};

/* Which framebuffer coordinate driver pixel `d` (of `size` along this axis)
 * reads - nodem-esp32's DriverView::pixel() axis mapping: a framebuffer
 * smaller than the driver's output once scaled is centered on it, then
 * everything is shifted by the mapping's offset. Shared by every driver, so
 * they all place and scale the framebuffer the same way. */
static inline int16_t nodem_map_axis(int d, int size, int fb_size, int scale, int shift)
{
	int margin = MAX((size - fb_size * scale) / 2, 0);
	int rel = d - margin;
	/* Floor division, so pixels left of/above the margin land on
	 * negative (off) framebuffer coordinates rather than rounding onto
	 * the first one. */
	int q = rel >= 0 ? rel / scale : -((-rel + scale - 1) / scale);

	return (int16_t)CLAMP(shift + q, INT16_MIN, INT16_MAX);
}

/* Framebuffer size in bytes - pixels packed contiguously, row after row,
 * MSB first, no per-row padding (see nodem-rs's Surface::draw_pixel). */
static inline uint32_t nodem_config_buffer_len(const struct nodem_config *c)
{
	return ((uint32_t)c->width * c->height * c->bits_per_pixel + 7) / 8;
}

static inline uint8_t nodem_expand(uint32_t v, int bits)
{
	/* Bit replication, so full scale maps to exactly 255. */
	switch (bits) {
	case 2:
		return v * 85;
	case 3:
		return (v << 5) | (v << 2) | (v >> 1);
	case 4:
		return v * 17;
	case 5:
		return (v << 3) | (v >> 2);
	case 6:
		return (v << 2) | (v >> 4);
	default:
		return v;
	}
}

/* Pixel `i` (row-major) of a framebuffer in the build's colour mode as
 * 8-bit r, g, b. Only meaningful for NODEM_COLOR builds. */
static inline void nodem_fb_rgb(const uint8_t *fb, uint32_t i, uint8_t rgb[3])
{
#if defined(NODEM_MODE_RGB332)
	uint8_t v = fb[i];

	rgb[0] = nodem_expand(v >> 5, 3);
	rgb[1] = nodem_expand((v >> 2) & 7, 3);
	rgb[2] = nodem_expand(v & 3, 2);
#elif defined(NODEM_MODE_RGB444)
	/* r, g, b nibbles one after the other, high nibble first. */
	for (int c = 0; c < 3; c++) {
		uint32_t n = i * 3 + c;
		uint8_t v = fb[n >> 1];

		rgb[c] = nodem_expand((n & 1) ? v & 0x0f : v >> 4, 4);
	}
#elif defined(NODEM_MODE_RGB555) || defined(NODEM_MODE_RGB565)
	uint16_t v = fb[i * 2] | (fb[i * 2 + 1] << 8);
#if defined(NODEM_MODE_RGB555)
	rgb[0] = nodem_expand((v >> 10) & 31, 5);
	rgb[1] = nodem_expand((v >> 5) & 31, 5);
#else
	rgb[0] = nodem_expand(v >> 11, 5);
	rgb[1] = nodem_expand((v >> 5) & 63, 6);
#endif
	rgb[2] = nodem_expand(v & 31, 5);
#elif defined(NODEM_MODE_RGB666)
	for (int c = 0; c < 3; c++) {
		rgb[c] = nodem_expand(fb[i * 3 + c] & 0x3f, 6);
	}
#elif defined(NODEM_MODE_RGB24)
	for (int c = 0; c < 3; c++) {
		rgb[c] = fb[i * 3 + c];
	}
#else
	rgb[0] = rgb[1] = rgb[2] = 0;
#endif
}

/* Pixel `i` as a lit/unlit dot - what a monochrome driver shows: the bit
 * itself at 1 bit/pixel, else lit when its brightness is at least half. */
static inline bool nodem_fb_lit(const uint8_t *fb, uint32_t i)
{
#if NODEM_COLOR
	uint8_t rgb[3];

	nodem_fb_rgb(fb, i, rgb);
	return (rgb[0] * 77 + rgb[1] * 151 + rgb[2] * 28) >= 128 * 256;
#else
	return fb[i >> 3] & (0x80 >> (i & 7));
#endif
}

/* Reads "nodem" back: a missing value (first boot) or an unparseable one is
 * replaced in config_store by the default, so "#cfg" shows what's actually
 * in use. */
void nodem_config_get(struct nodem_config *out);

/* Validates and persists "nodem" (normalized; an empty value means the
 * default, as in nodem-esp32's "#nodem") and has nodem_task.c apply it
 * right away. Returns 0 on success, -EINVAL for a malformed value (nothing
 * stored), or config_store's error if persisting it failed. */
int nodem_config_set(const char *value);

/* Bumped by every successful nodem_config_set() - nodem_task.c re-reads the
 * config whenever this no longer matches what it last applied. */
uint32_t nodem_config_generation(void);

#endif /* NODEM_CONFIG_H_ */
