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

/* 1 bit/pixel, so e.g. 256x128 or 512x64 - bounded by RAM: nodem_task.c's
 * framebuffer and display_task.c's copy of it are both this size. */
#define NODEM_FB_MAX_SIZE 4096

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
