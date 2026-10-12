#include "hub75_task.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/devicetree.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include <hal/nrf_gpio.h>
#include <hal/nrf_pwm.h>

#include "config_store.h"
#include "display_task.h"
#include "log.h"

#define TAG "hub75"

#define HUB75_STACK_SIZE 1024
#define HUB75_PRIORITY   4

#define HUB75_KEY       "hub75"
/* config_store's MAX_STR_LEN (127) + NUL. */
#define HUB75_VALUE_LEN 128

/* P0.00-P0.07 carry the HUB75 connector's odd pins in order, P0.08-P0.13
 * its even ones (1/2 R1/G1, 3/5/7 B1/R2/B2, 6/8 G2/E, ...) - see
 * hub75_task.h - so a straight two-row ribbon routes them. */
#define PIN_R1  0
#define PIN_B1  1
#define PIN_R2  2
#define PIN_B2  3
#define PIN_A   4
#define PIN_C   5
#define PIN_CLK 6
#define PIN_OE  7
#define PIN_G1  8
#define PIN_G2  9
#define PIN_E   10
#define PIN_B   11
#define PIN_D   12
#define PIN_LAT 13

/* main.c's heartbeat LED (LED1) shares P0.00 with R1: while the panel
 * runs, its toggling only reaches the pin between rows, while CLK is low
 * (PWM1 drives it otherwise), so it never gets shifted in - and while the
 * panel is disabled, the pin goes back to being a plain output for it. */
#define HEARTBEAT_PIN NRF_DT_GPIOS_TO_PSEL(DT_ALIAS(led0), gpios)

/* In address bit order - a 1/8 scan panel only uses the first three, 1/16
 * the first four. The rest are left alone: on many panels with fewer row
 * addresses, HUB75 pin 8 (E) is GND. */
static const uint8_t addr_pins[] = { PIN_A, PIN_B, PIN_C, PIN_D, PIN_E };

static const uint32_t pins_a[NRF_PWM_CHANNEL_COUNT] = { PIN_R1, PIN_G1, PIN_B1, PIN_CLK };
static const uint32_t pins_b[NRF_PWM_CHANNEL_COUNT] = { PIN_R2, PIN_G2, PIN_B2, PIN_OE };

#define PWM_A NRF_PWM1
#define PWM_B NRF_PWM2
#define PWM_A_NODE DT_NODELABEL(pwm1)
#define PWM_A_IRQ_PRIORITY 1

/* The PWM's 16MHz base clock, in kHz, and the pixel clock range a config
 * may ask for: one PWM period per shifted column, an even number of ticks
 * (CLK rises halfway through) - so 1000kHz is 16 ticks. Faster isn't
 * offered: the ISR renders one row per row shifted (see render_row()), so
 * its CPU share grows in proportion - ~17% at 500kHz on a 64x32 1/8
 * scan panel, measured with a TIMER. */
#define PWM_BASE_KHZ  16000
#define CLOCK_MIN_KHZ 100
#define CLOCK_MAX_KHZ 1000

/* Bit 15 set = the period starts high and drops at the compare value:
 * compare 0 never goes high, a compare value past COUNTERTOP never drops. */
#define PWM_LOW  (BIT(15) | 0)
#define PWM_HIGH (BIT(15) | 0x7FFF)
/* Bit 15 clear = starts low and rises at the compare value - used for CLK
 * at half the period (the rising edge the panel's shift registers clock on
 * lands mid-period, with the data lines stable for half a period on either
 * side of it) and for OE's one partial step (lit for that many ticks). */

/* Bits per colour channel the driver can do - one bit plane each - and
 * what a config (or preset) that doesn't say gets: 4 keeps two chained
 * 64x32 1/8 scan panels at 60Hz; every bit more halves the refresh rate. */
#define MAX_DEPTH     8
#define DEPTH_DEFAULT 4

/* Where every buffer whose size depends on the config comes from - the
 * frame, the PWM step buffers, the chain and column tables, the effects'
 * colour tables - laid out per config by hub75_alloc(). A config that
 * doesn't fit is driven on fewer of its panels (see hub75_alloc()). Two
 * 64x32 1/8 scan panels at 4 bits per channel take ~21KB of it. Only ever
 * one block at a time, so a plain arena rather than a heap - no allocator
 * code, no fragmentation. */
#define HUB75_ARENA_SIZE (22 * 1024)
static uint8_t hub75_arena[HUB75_ARENA_SIZE] __aligned(8);

/* Two idle steps (CLK low, OE high) after the data: SEQEND fires once the last
 * value has been *read* from RAM, which can be before the previous one has
 * finished playing, and the SEQEND->STOP short only takes effect at the end
 * of the current period - with two trailing idle steps, every data period
 * has fully played by the time the PWM stops. */
#define IDLE_STEPS    2

/* The column driver chips - what has to happen before a panel shows
 * anything. */
enum hub75_chip {
	/* Plain shift registers (ICN2037/ICN2038, MBI5124, DP5125, ...) -
	 * nothing to set up. */
	CHIP_SHIFT,
	/* FM6126A, FM6124, ICN2038S - stay dark until two configuration
	 * registers are written, see chip_init_fm6126a(). */
	CHIP_FM6126A,
};

static const struct {
	const char *name;
	enum hub75_chip chip;
} chip_names[] = {
	/* The first name of each chip is what's stored. */
	{ "shift", CHIP_SHIFT },
	{ "fm6126a", CHIP_FM6126A },
	{ "fm6124", CHIP_FM6126A },
	{ "icn2038s", CHIP_FM6126A },
};

static const char *chip_name(enum hub75_chip chip)
{
	for (size_t i = 0; i < ARRAY_SIZE(chip_names); i++) {
		if (chip_names[i].chip == chip) {
			return chip_names[i].name;
		}
	}
	return "?";
}

/* What a colour field paints with: one fixed colour, or an effect - see
 * effect_at() for what each looks like. */
enum hub75_effect {
	EFFECT_NONE,
	EFFECT_PLASMA,
	EFFECT_STRIPES,
	EFFECT_WAVE,
	EFFECT_SCROLL,
	EFFECT_CYCLE,
	EFFECT_FADE,
	EFFECT_FIRE,
	EFFECT_SPARKLE,
	EFFECT_RIPPLE,
	EFFECT_GRADIENT,
};

#define EFFECT_SPEED_MAX 32

static const struct {
	const char *name;
	enum hub75_effect effect;
	/* How far it moves per frame (FX_FRAME_MS), by default - 0 for one
	 * that doesn't move (and takes no speed). */
	uint8_t speed;
} effects[] = {
	{ "plasma", EFFECT_PLASMA, 2 },
	/* What "plasma" used to be called - still read from stored configs,
	 * stored as "plasma" from then on (the first name is what's
	 * stored). */
	{ "rainbow", EFFECT_PLASMA, 2 },
	{ "stripes", EFFECT_STRIPES, 4 },
	{ "wave", EFFECT_WAVE, 4 },
	{ "scroll", EFFECT_SCROLL, 3 },
	{ "cycle", EFFECT_CYCLE, 3 },
	{ "fade", EFFECT_FADE, 2 },
	{ "fire", EFFECT_FIRE, 4 },
	{ "sparkle", EFFECT_SPARKLE, 4 },
	{ "ripple", EFFECT_RIPPLE, 4 },
	{ "gradient", EFFECT_GRADIENT, 0 },
};

#define PALETTE_MAX   8
#define SCALE_MAX     32
/* The pattern's size as it was before <scale> existed. */
#define SCALE_DEFAULT 8

struct hub75_color {
	enum hub75_effect effect;
	/* Effects only: per-frame step, 1-EFFECT_SPEED_MAX (0 for a static
	 * one). */
	uint8_t speed;
	/* Effects only: the 3-bit colours to use, in order (palette_len 0:
	 * the effect's own - see effect_palette()), and the pattern's size,
	 * 1-SCALE_MAX (SCALE_DEFAULT: as designed, larger: stretched). */
	uint8_t palette[PALETTE_MAX];
	uint8_t palette_len;
	uint8_t scale;
	/* EFFECT_NONE only: 0xRRGGBB. */
	uint32_t rgb;
};

/* Palette letters, the fully saturated colours (and black, white) - in
 * 3-bit form, bit 0 red, bit 1 green, bit 2 blue. */
static const char palette_letters[] = "krgybmcw";

struct hub75_config {
	/* The whole display - a grid of `grid_x` by `grid_y` panels, each
	 * width / grid_x by height / grid_y. */
	uint16_t width;
	uint16_t height;
	/* Layout "r"/"b": the whole display mirrored, so the chain enters at
	 * the right/bottom (seen from the front) - see frame_locate_row()
	 * for what that, `grid_*` and `serpentine` mean for the chain. */
	bool flip_x;
	bool flip_y;
	uint8_t grid_x;
	uint8_t grid_y;
	/* Every second row of panels upside down, the chain running back
	 * through it. */
	bool serpentine;
	/* Row addresses: 2, 4, 8, 16 or 32. */
	uint8_t scan;
	/* How each row address's chain runs through the rows that share it
	 * (more than one when there are fewer row addresses than rows per
	 * half), in pieces of `segment` columns (0: a whole panel width),
	 * one piece per row in turn - the first row's nearest the input, or
	 * furthest ("f", far_first) - every second piece backwards if
	 * `zigzag`. See chain_build(). */
	bool far_first;
	uint8_t segment;
	bool zigzag;
	uint16_t clock_khz;
	enum hub75_chip chip;
	/* Bits per colour channel, 1-MAX_DEPTH - bit planes, see hub75_isr(). */
	uint8_t depth;
	struct hub75_color color;
	struct hub75_color off_color;
	uint16_t brightness;
};

/* --- config --- */

/* Bumped by hub75_config_set() - hub75_task() re-reads the config and
 * restarts the panel with it whenever this no longer matches the generation
 * it last applied. hub75_wake is given by that and by every frame
 * submitted. */
static atomic_t hub75_generation = ATOMIC_INIT(0);
static K_SEM_DEFINE(hub75_wake, 0, 1);

/* The next `sep`-separated field of `*s` into `out` (trimmed of spaces),
 * advancing `*s` past it and its separator - NULL once there's no more. An
 * empty `out` for an empty field or none left. False if it doesn't fit. */
static bool next_field(const char **s, char sep, char *out, size_t out_len)
{
	out[0] = '\0';
	if (*s == NULL) {
		return true;
	}

	const char *end = strchr(*s, sep);
	size_t len = end ? (size_t)(end - *s) : strlen(*s);
	const char *start = *s;

	*s = end ? end + 1 : NULL;

	while (len > 0 && *start == ' ') {
		start++;
		len--;
	}
	while (len > 0 && start[len - 1] == ' ') {
		len--;
	}
	if (len >= out_len) {
		return false;
	}
	memcpy(out, start, len);
	out[len] = '\0';
	return true;
}

/* A plain decimal number in min..max - false for anything else. */
static bool parse_num(const char *s, long min, long max, long *out)
{
	char *end;
	long v = strtol(s, &end, 10);

	if (end == s || *end != '\0' || v < min || v > max) {
		return false;
	}
	*out = v;
	return true;
}

static bool parse_layout(const char *s, struct hub75_config *c)
{
	bool have_x = false, have_y = false;

	c->flip_x = false;
	c->flip_y = false;
	for (; *s; s++) {
		if ((*s == 'l' || *s == 'r') && !have_x) {
			c->flip_x = *s == 'r';
			have_x = true;
		} else if ((*s == 't' || *s == 'b') && !have_y) {
			c->flip_y = *s == 'b';
			have_y = true;
		} else {
			return false;
		}
	}
	return true;
}

/* "<cols>[x<rows>][i]" - a lone number is that many panels side by side. */
static bool parse_grid(const char *s, struct hub75_config *c)
{
	char *end;
	long cols = strtol(s, &end, 10);
	long rows = 1;

	if (end == s || cols < 1 || cols > 16) {
		return false;
	}
	if (*end == 'x') {
		s = end + 1;
		rows = strtol(s, &end, 10);
		if (end == s || rows < 1 || rows > 16) {
			return false;
		}
	}
	if (*end == 'i') {
		c->serpentine = true;
		end++;
	}
	if (*end != '\0' || cols * rows > 16) {
		return false;
	}
	c->grid_x = cols;
	c->grid_y = rows;
	return true;
}

static bool parse_geometry(const char *s, struct hub75_config *c)
{
	char field[16];
	long width, height;

	if (!next_field(&s, ':', field, sizeof(field)) ||
	    !parse_num(field, 1, HUB75_MAX_PIXELS, &width)) {
		return false;
	}
	if (!next_field(&s, ':', field, sizeof(field)) ||
	    !parse_num(field, 1, HUB75_MAX_PIXELS, &height)) {
		return false;
	}
	if (!next_field(&s, ':', field, sizeof(field)) || !parse_layout(field, c)) {
		return false;
	}
	c->grid_x = 1;
	c->grid_y = 1;
	c->serpentine = false;
	if (!next_field(&s, ':', field, sizeof(field)) || s != NULL) {
		return false;
	}
	if (field[0] != '\0' && !parse_grid(field, c)) {
		return false;
	}
	c->width = width;
	c->height = height;
	return true;
}

/* "<n|f>[<segment>][z]" - see struct hub75_config. */
static bool parse_chain(const char *s, struct hub75_config *c)
{
	if (*s != 'n' && *s != 'f') {
		return false;
	}
	c->far_first = *s++ == 'f';
	c->segment = 0;
	c->zigzag = false;

	if (*s >= '1' && *s <= '9') {
		char *end;
		long v = strtol(s, &end, 10);

		if (v > UINT8_MAX) {
			return false;
		}
		c->segment = v;
		s = end;
	}
	if (*s == 'z') {
		c->zigzag = true;
		s++;
	}
	return *s == '\0';
}

static bool parse_protocol(const char *s, struct hub75_config *c)
{
	static const struct {
		const char *name;
		uint8_t scan;
		bool far_first;
	} presets[] = {
		{ "4s", 4, false },
		{ "8s", 8, false },
		{ "8sf", 8, true },
		{ "16s", 16, false },
		{ "32s", 32, false },
	};

	for (size_t i = 0; i < ARRAY_SIZE(presets); i++) {
		if (strcmp(s, presets[i].name) == 0) {
			c->scan = presets[i].scan;
			c->far_first = presets[i].far_first;
			c->segment = 0;
			c->zigzag = false;
			c->clock_khz = 500;
			c->chip = CHIP_SHIFT;
			c->depth = DEPTH_DEFAULT;
			return true;
		}
	}

	char field[16];
	long v;

	/* "<scan>:<chain>:<clock_khz>:<chip>:<depth>", each part optional -
	 * c already holds the default's. */
	if (!next_field(&s, ':', field, sizeof(field))) {
		return false;
	}
	if (field[0] != '\0') {
		if (!parse_num(field, 2, 32, &v) || (v & (v - 1)) != 0) {
			return false;
		}
		c->scan = v;
	}
	if (!next_field(&s, ':', field, sizeof(field))) {
		return false;
	}
	if (field[0] != '\0' && !parse_chain(field, c)) {
		return false;
	}
	if (!next_field(&s, ':', field, sizeof(field))) {
		return false;
	}
	if (field[0] != '\0') {
		if (!parse_num(field, CLOCK_MIN_KHZ, CLOCK_MAX_KHZ, &v)) {
			return false;
		}
		c->clock_khz = v;
	}
	if (!next_field(&s, ':', field, sizeof(field))) {
		return false;
	}
	if (field[0] != '\0') {
		size_t i;

		for (i = 0; i < ARRAY_SIZE(chip_names); i++) {
			if (strcmp(field, chip_names[i].name) == 0) {
				c->chip = chip_names[i].chip;
				break;
			}
		}
		if (i == ARRAY_SIZE(chip_names)) {
			return false;
		}
	}
	if (!next_field(&s, ':', field, sizeof(field)) || s != NULL) {
		return false;
	}
	if (field[0] != '\0') {
		if (!parse_num(field, 1, MAX_DEPTH, &v)) {
			return false;
		}
		c->depth = v;
	}
	return true;
}

/* "<effect>[:<speed>[:<palette>[:<scale>]]]" - one of `effects`, speed
 * 1-EFFECT_SPEED_MAX (none for a static one), palette up to PALETTE_MAX of
 * `palette_letters`, scale 1-SCALE_MAX, an empty part its default - or "#"
 * and up to 6 hex digits, 2 per channel, r then g then b - missing digits
 * are 0, same as nodem-esp32's "#iled" colours ("#f" is 0xf00000). */
static bool parse_color(const char *s, struct hub75_color *out)
{
	if (s[0] != '#') {
		char name[12], speed[4], palette[PALETTE_MAX + 1], scale[4];

		if (!next_field(&s, ':', name, sizeof(name)) ||
		    !next_field(&s, ':', speed, sizeof(speed)) ||
		    !next_field(&s, ':', palette, sizeof(palette)) ||
		    !next_field(&s, ':', scale, sizeof(scale)) || s != NULL) {
			return false;
		}

		size_t i;

		for (i = 0; i < ARRAY_SIZE(effects); i++) {
			if (strcmp(name, effects[i].name) == 0) {
				break;
			}
		}
		if (i == ARRAY_SIZE(effects)) {
			return false;
		}

		struct hub75_color c = {
			.effect = effects[i].effect,
			.speed = effects[i].speed,
			.scale = SCALE_DEFAULT,
		};
		long v;

		if (speed[0] != '\0') {
			if (effects[i].speed == 0 || !parse_num(speed, 1, EFFECT_SPEED_MAX, &v)) {
				return false;
			}
			c.speed = v;
		}
		for (const char *l = palette; *l; l++) {
			const char *at = strchr(palette_letters, *l);

			if (!at) {
				return false;
			}
			c.palette[c.palette_len++] = at - palette_letters;
		}
		if (scale[0] != '\0') {
			if (!parse_num(scale, 1, SCALE_MAX, &v)) {
				return false;
			}
			c.scale = v;
		}
		*out = c;
		return true;
	}
	if (strlen(s) > 7) {
		return false;
	}

	uint32_t v = 0;
	size_t n = 0;

	for (s++; *s; s++, n++) {
		char ch = *s;
		uint32_t d;

		if (ch >= '0' && ch <= '9') {
			d = ch - '0';
		} else if (ch >= 'a' && ch <= 'f') {
			d = ch - 'a' + 10;
		} else if (ch >= 'A' && ch <= 'F') {
			d = ch - 'A' + 10;
		} else {
			return false;
		}
		v |= d << (20 - 4 * n);
	}
	*out = (struct hub75_color){ .effect = EFFECT_NONE, .rgb = v };
	return true;
}

/* HUB75_DEFAULT, parsed - what every empty or missing field falls back
 * to. */
static void hub75_default(struct hub75_config *c)
{
	*c = (struct hub75_config){
		.width = 64,
		.height = 32,
		.flip_x = true,
		.grid_x = 1,
		.grid_y = 1,
		.scan = 8,
		.clock_khz = 500,
		.chip = CHIP_SHIFT,
		.depth = DEPTH_DEFAULT,
		.color = { .effect = EFFECT_NONE, .rgb = 0xff0000 },
		.off_color = { .effect = EFFECT_NONE, .rgb = 0x000000 },
		.brightness = 512,
	};
}

/* 1 (enabled, `out` filled in), 0 (disabled: an empty value) or -EINVAL.
 * Every empty or missing field keeps HUB75_DEFAULT's. */
static int hub75_config_parse(const char *value, struct hub75_config *out)
{
	while (*value == ' ') {
		value++;
	}
	if (*value == '\0') {
		return 0;
	}

	struct hub75_config c;
	const char *s = value;
	char field[32];
	long v;

	hub75_default(&c);

	if (!next_field(&s, ',', field, sizeof(field)) ||
	    (field[0] != '\0' && !parse_geometry(field, &c))) {
		return -EINVAL;
	}
	if (!next_field(&s, ',', field, sizeof(field)) ||
	    (field[0] != '\0' && !parse_protocol(field, &c))) {
		return -EINVAL;
	}
	if (!next_field(&s, ',', field, sizeof(field)) ||
	    (field[0] != '\0' && !parse_color(field, &c.color))) {
		return -EINVAL;
	}
	if (!next_field(&s, ',', field, sizeof(field)) ||
	    (field[0] != '\0' && !parse_color(field, &c.off_color))) {
		return -EINVAL;
	}
	if (!next_field(&s, ',', field, sizeof(field)) ||
	    (field[0] != '\0' && !parse_num(field, 0, HUB75_BRIGHTNESS_MAX, &v))) {
		return -EINVAL;
	}
	if (field[0] != '\0') {
		c.brightness = v;
	}
	if (s != NULL) {
		return -EINVAL;
	}

	/* The panels split the display evenly; on each, every data line set
	 * (R1G1B1, R2G2B2) drives half the rows, a whole number of
	 * row-address blocks of them, and the chain's pieces split its width
	 * evenly. The chain runs through every panel: grid_x * grid_y panel
	 * widths per row block. */
	uint16_t panel_w = c.width / c.grid_x;
	uint16_t panel_h = c.height / c.grid_y;

	if ((uint32_t)c.width * c.height > HUB75_MAX_PIXELS || c.width % c.grid_x != 0 ||
	    c.height % c.grid_y != 0 || panel_h % (2 * c.scan) != 0 ||
	    (c.segment && panel_w % c.segment != 0)) {
		return -EINVAL;
	}

	*out = c;
	return 1;
}

/* The inverse of parse_color() - an effect's parts only as far as one
 * isn't its default (an earlier part empty if it is, but a later one
 * isn't: "plasma::cg"). */
static void format_color(const struct hub75_color *c, char *buf, size_t buf_len)
{
	if (c->effect == EFFECT_NONE) {
		snprintf(buf, buf_len, "#%06x", (unsigned int)c->rgb);
		return;
	}
	for (size_t i = 0; i < ARRAY_SIZE(effects); i++) {
		if (effects[i].effect != c->effect) {
			continue;
		}

		char speed[4] = "", palette[PALETTE_MAX + 1] = "", scale[4] = "";

		if (c->speed != effects[i].speed) {
			snprintf(speed, sizeof(speed), "%u", c->speed);
		}
		for (size_t j = 0; j < c->palette_len; j++) {
			palette[j] = palette_letters[c->palette[j]];
		}
		palette[c->palette_len] = '\0';
		if (c->scale != SCALE_DEFAULT) {
			snprintf(scale, sizeof(scale), "%u", c->scale);
		}

		if (scale[0]) {
			snprintf(buf, buf_len, "%s:%s:%s:%s", effects[i].name, speed, palette, scale);
		} else if (palette[0]) {
			snprintf(buf, buf_len, "%s:%s:%s", effects[i].name, speed, palette);
		} else if (speed[0]) {
			snprintf(buf, buf_len, "%s:%s", effects[i].name, speed);
		} else {
			snprintf(buf, buf_len, "%s", effects[i].name);
		}
		return;
	}
}

/* Every part spelled out, presets expanded, so "#cfg" shows exactly what's
 * in use - same idea as nodem-esp32's "#iled" layout_to_csv(). */
static void hub75_config_format(const struct hub75_config *c, char *buf, size_t buf_len)
{
	char chain[8];

	if (c->segment) {
		snprintf(chain, sizeof(chain), "%c%u%s", c->far_first ? 'f' : 'n', c->segment,
			 c->zigzag ? "z" : "");
	} else {
		snprintf(chain, sizeof(chain), "%c%s", c->far_first ? 'f' : 'n',
			 c->zigzag ? "z" : "");
	}
	char color[32], off_color[32];

	format_color(&c->color, color, sizeof(color));
	format_color(&c->off_color, off_color, sizeof(off_color));
	snprintf(buf, buf_len, "%u:%u:%c%c:%ux%u%s,%u:%s:%u:%s:%u,%s,%s,%u", c->width,
		 c->height, c->flip_x ? 'r' : 'l', c->flip_y ? 'b' : 't', c->grid_x, c->grid_y,
		 c->serpentine ? "i" : "", c->scan, chain, c->clock_khz, chip_name(c->chip),
		 c->depth, color, off_color, c->brightness);
}

int hub75_config_set(const char *value)
{
	struct hub75_config config;
	char normalized[HUB75_VALUE_LEN];
	int parsed = hub75_config_parse(value, &config);

	if (parsed < 0) {
		return parsed;
	}
	if (parsed > 0) {
		hub75_config_format(&config, normalized, sizeof(normalized));
		value = normalized;
	} else {
		value = "";
	}

	int err = config_set_str(HUB75_KEY, value);

	if (!err) {
		atomic_inc(&hub75_generation);
		k_sem_give(&hub75_wake);
	}
	return err;
}

/* Reads "hub75" back, the same way display_task.c's oled_config_read()
 * does: a missing value (first boot) or a malformed one is replaced in
 * config_store by HUB75_DEFAULT (so "#cfg" shows what's actually in use); a
 * deliberately disabled ("") value is kept as it is. False if disabled. */
static bool hub75_config_read(struct hub75_config *out)
{
	/* Distinguishes "never set" from a deliberately stored "" (disabled),
	 * which config_get_str()'s default alone can't. */
	static const char unset[] = "\x01";
	char value[HUB75_VALUE_LEN];

	config_get_str(HUB75_KEY, value, sizeof(value), unset);

	if (strcmp(value, unset) != 0) {
		int parsed = hub75_config_parse(value, out);

		if (parsed >= 0) {
			return parsed > 0;
		}
		warn(TAG, "malformed '%s' in config ('%s'), falling back to default", HUB75_KEY,
		     value);
	}

	if (config_set_str(HUB75_KEY, HUB75_DEFAULT)) {
		error(TAG, "failed to persist default '%s'", HUB75_KEY);
	}
	return hub75_config_parse(HUB75_DEFAULT, out) > 0;
}

/* --- driver --- */

/* The running config's geometry, as the ISR and the frame_draw_*()
 * functions need it - only ever changed while the panel is stopped
 * (hub75_start()). */
static struct hub75_config active;
/* Row addresses, how many rows of each half share one (1 on a 1/16 scan
 * 64x32 panel, 2 on a 1/8 scan one), and so how many columns each row
 * address's chain shifts: all of those rows, one panel width each. */
static uint8_t scan_rows;
static uint8_t row_blocks;
static uint16_t chain_len;
/* One panel's size, and how many there are and are driven - the first
 * drive_panels along the chain, all of them unless hub75_alloc() couldn't
 * get the memory - and so the columns per shift that are actually data
 * (the rest is shifted out as blank filler, see pwm_setup()). */
static uint16_t panel_w;
static uint16_t panel_h;
static uint8_t total_panels;
static uint8_t drive_panels;
static uint16_t drive_len;
static uint32_t addr_all_mask;

/* EasyDMA source for each instance - one 16-bit value per channel per step
 * (Individual decoder mode), kept as two 32-bit words per step - channels
 * 0|1 and 2|3, little-endian - so render_row() writes a whole step in two
 * stores. Two of each, ping-ponged: while one pair is being shifted out,
 * the ISR renders the next row into the other. seq_b's channel 3 is OE:
 * while one row is being shifted in, it gates the previously latched one.
 * drive_len + IDLE_STEPS steps each, from hub75_arena. */
typedef uint32_t step_t[2];
static step_t *seq_a[2];
static step_t *seq_b[2];
/* The blank filler for the panels that aren't driven - one step, repeated
 * (PWM "refresh") for every column of theirs. */
static step_t filler_a;
static step_t filler_b;

#define STEP_WORD(lo, hi) ((uint32_t)(lo) | ((uint32_t)(hi) << 16))

/* Per `frame` byte (top colour in bits 0-2, bottom in 3-5 - bit 0 red,
 * bit 1 green, bit 2 blue, each), that step's four words: seq_a's R1|G1
 * and B1|CLK, seq_b's R2|G2 and B2 with an empty high half, where OE goes.
 * One lookup per column is all render_span() does. */
#define PX_COMBOS 64
static uint32_t lut_px[PX_COMBOS][4];

/* OE from the brightness, per bit plane: lit (low) for the first
 * `oe_full_steps` data steps of the shift that follows the plane's, then
 * `oe_partial` (already in the high half: lit for part of that step, or
 * blanked) for the next one, blanked after. Plane k gets 2^k units of
 * light - the most significant one the whole configured brightness. */
static uint16_t oe_full_steps[MAX_DEPTH];
static uint32_t oe_partial[MAX_DEPTH];
/* Bit planes of the running config. */
static uint8_t depth;
/* PWM ticks per column, for the running config. */
static uint16_t pwm_top;

/* OE for `brightness` (0-HUB75_BRIGHTNESS_MAX) - takes effect from the next
 * row rendered, so it can change while the panel runs.
 * Both halves at once, under irq_lock(), so render_row() never sees one
 * without the other. */
static void oe_set(uint16_t brightness)
{
	uint32_t row_ticks = (uint32_t)drive_len * pwm_top;
	uint32_t on_ticks = (row_ticks * brightness + HUB75_BRIGHTNESS_MAX / 2) /
			    HUB75_BRIGHTNESS_MAX;
	uint16_t full[MAX_DEPTH];
	uint32_t partial[MAX_DEPTH];

	for (int k = 0; k < depth; k++) {
		uint32_t ticks = (on_ticks << k) >> (depth - 1);
		uint16_t rest = ticks % pwm_top;

		full[k] = ticks / pwm_top;
		partial[k] = STEP_WORD(0, rest ? rest : PWM_HIGH);
	}

	unsigned int key = irq_lock();

	memcpy(oe_full_steps, full, sizeof(full));
	memcpy(oe_partial, partial, sizeof(partial));
	irq_unlock(key);
}

/* What's on the panel, per row address, in shift order (index 0 shifted
 * first): `depth` bytes per chain position, byte k bit plane k - bit k of
 * the top half's (R1G1B1) colour channel levels in its bits 0-2, the
 * bottom half's (R2G2B2) in bits 3-5 - just what one shift needs, in one
 * place. scan_rows rows of drive_len positions each, from hub75_arena;
 * written through frame_locate_row()/frame_row_set(). */
static uint8_t *frame;

/* The shift currently in progress - latched and lit by the next interrupt
 * - as an index into the cycle every row address goes through, plane by
 * plane (row = shift / depth, plane = shift % depth), and which
 * seq_a/seq_b pair it's in. */
static uint16_t shift_seq;
static uint8_t shift_buf;

static bool running;

/* What hub75_stat_format() reports - set by hub75_task() (and
 * hub75_start()), read from the command handler's thread; plain stores of
 * small values, so a reader at worst sees the old state or the new one. */
enum hub75_stat {
	STAT_DISABLED,
	STAT_OK,
	STAT_LOW_MEM,
	STAT_NO_MEM,
	STAT_OVERLOAD,
};
static volatile uint8_t stat_state;
static volatile uint16_t stat_width;
static volatile uint16_t stat_height;
static volatile uint8_t stat_driven;
static volatile uint8_t stat_panels;

size_t hub75_stat_format(char *buf, size_t buf_len)
{
	static const char *const states[] = {
		[STAT_OK] = "ok",
		[STAT_NO_MEM] = "no_mem",
		[STAT_OVERLOAD] = "overload",
	};
	uint8_t state = stat_state;

	if (state == STAT_DISABLED) {
		return snprintf(buf, buf_len, "hub75,0,,");
	}
	if (state == STAT_LOW_MEM) {
		return snprintf(buf, buf_len, "hub75,1,%ux%u,low_mem %u/%u panels", stat_width,
				stat_height, stat_driven, stat_panels);
	}
	return snprintf(buf, buf_len, "hub75,1,%ux%u,%s", stat_width, stat_height,
			states[state]);
}

/* Set by hub75_isr() when it can't keep the panel going - see
 * hub75_overload_trip() - for hub75_task() to stop it and say why. */
#define OVERLOAD_NONE    0
#define OVERLOAD_OVERRUN 1
#define OVERLOAD_STUCK   2
static atomic_t hub75_overload = ATOMIC_INIT(OVERLOAD_NONE);
/* Rows in a row whose render didn't finish before the next row had already
 * been shifted out (so the next interrupt was due before this one ended). */
static uint16_t overruns;
#define OVERRUN_LIMIT 32
/* What hub75_task() does about a trip - see there. */
#define OVERLOAD_RESTART_MS 200
#define OVERLOAD_TRIPS      3
#define OVERLOAD_WINDOW_MS  10000
/* How many times hub75_isr() checks for PWM_B to have stopped too before
 * giving up - it's never more than a tick or two behind PWM_A, so this is
 * far beyond anything legitimate (some 100us). */
#define STOP_SPIN_LIMIT 2000

static void pins_set(uint32_t mask)
{
	NRF_P0->OUTSET = mask;
}

static void pins_clear(uint32_t mask)
{
	NRF_P0->OUTCLR = mask;
}

/* High drive for every HUB75 output: jumper wires plus the panel's input
 * buffer are a much bigger load than standard drive is meant for, enough
 * to round off short pulses before they reach the panel's input-high
 * threshold. */
static void pin_cfg_output(uint32_t pin, bool high)
{
	if (high) {
		nrf_gpio_pin_set(pin);
	} else {
		nrf_gpio_pin_clear(pin);
	}
	nrf_gpio_cfg(pin, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT,
		     NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_H0H1, NRF_GPIO_PIN_NOSENSE);
}

static uint32_t addr_mask(uint8_t row)
{
	uint32_t mask = 0;

	for (size_t i = 0; i < ARRAY_SIZE(addr_pins); i++) {
		if (row & BIT(i)) {
			mask |= BIT(addr_pins[i]);
		}
	}
	return mask;
}

/* Colours on their way to the panel: 8 bits per channel, red in bits 0-7,
 * green 8-15, blue 16-23 - already gamma-corrected (see gamma8), so linear
 * in light; spread() takes them down to the panel's depth. */
#define RGB24(r, g, b) ((uint32_t)(r) | ((uint32_t)(g) << 8) | ((uint32_t)(b) << 16))

/* Gamma 2 - (v / 255)^2, close enough to the usual 2.2 - so "#808080" (or
 * halfway through a blend) looks like about half brightness, not barely
 * dimmer than full. Computed, not a table: flash is tight. */
static inline uint8_t gamma8(uint8_t v)
{
	return (v * v + 254) / 255;
}

static uint32_t color_rgb24(uint32_t rgb)
{
	return RGB24(gamma8(rgb >> 16), gamma8(rgb >> 8), gamma8(rgb));
}

/* RGB24() `rgb` the way `frame` holds it: each channel rounded to the
 * running depth's 2^depth levels, bit k of every channel's level into
 * byte k (bit plane k) - red bit 0, green 1, blue 2. */
static uint64_t spread(uint32_t rgb)
{
	const uint32_t top = (1u << depth) - 1;
	uint32_t level[3];
	uint64_t planes = 0;

	for (int ch = 0; ch < 3; ch++) {
		level[ch] = (((rgb >> (8 * ch)) & 0xff) * top + 127) / 255;
	}
	for (int k = 0; k < depth; k++) {
		uint64_t bits = ((level[0] >> k) & 1) | (((level[1] >> k) & 1) << 1) |
				(((level[2] >> k) & 1) << 2);

		planes |= bits << (8 * k);
	}
	return planes;
}

/* --- colour effects, redrawn every FX_FRAME_MS while one is in use --- */

#define FX_FRAME_MS 40

/* sin() over a quarter wave, 64 steps to it, scaled to +-127. */
static const int8_t sin_quarter[65] = {
	0,   3,   6,   9,   12,  16,  19,  22,  25,  28,  31,  34,  37,  40,  43,  46,  49,
	51,  54,  57,  60,  63,  65,  68,  71,  73,  76,  78,  81,  83,  85,  88,  90,  92,
	94,  96,  98,  100, 102, 104, 106, 107, 109, 111, 112, 113, 115, 116, 117, 118, 120,
	121, 122, 122, 123, 124, 125, 125, 126, 126, 126, 127, 127, 127, 127,
};

/* sin(a * 2pi / 256) * 127. */
static int sin8(uint8_t a)
{
	uint8_t i = a & 63;

	switch (a >> 6) {
	case 0:
		return sin_quarter[i];
	case 1:
		return sin_quarter[64 - i];
	case 2:
		return -sin_quarter[i];
	default:
		return -sin_quarter[64 - i];
	}
}

/* The effects' own palettes: the fully saturated colours around the
 * colour wheel (red, yellow, green, cyan, blue, magenta) for most, black to
 * white by heat for fire, the wheel and white for sparkle. */
static const uint8_t wheel[] = { 0x1, 0x3, 0x2, 0x6, 0x4, 0x5 };
static const uint8_t fire_heat[] = { 0x0, 0x1, 0x3, 0x7 };
static const uint8_t sparkle_pool[] = { 0x1, 0x3, 0x2, 0x6, 0x4, 0x5, 0x7 };

static void effect_palette(const struct hub75_color *c, const uint8_t **pal, uint8_t *n)
{
	if (c->palette_len) {
		*pal = c->palette;
		*n = c->palette_len;
	} else if (c->effect == EFFECT_FIRE) {
		*pal = fire_heat;
		*n = ARRAY_SIZE(fire_heat);
	} else if (c->effect == EFFECT_SPARKLE) {
		*pal = sparkle_pool;
		*n = ARRAY_SIZE(sparkle_pool);
	} else {
		*pal = wheel;
		*n = ARRAY_SIZE(wheel);
	}
}

/* A cheap integer hash (lowbias32) - fire and sparkle's randomness,
 * repeatable for the same inputs, so they need no per-pixel state. */
static inline uint32_t hash3(uint32_t x, uint32_t y, uint32_t z)
{
	uint32_t h = x * 0x9E3779B1u ^ y * 0x85EBCA77u ^ z * 0xC2B2AE3Du;

	h ^= h >> 16;
	h *= 0x7feb352du;
	h ^= h >> 15;
	h *= 0x846ca68bu;
	h ^= h >> 16;
	return h;
}

/* Effect frames drawn so far - advanced by hub75_task() every FX_FRAME_MS
 * while an effect that moves is in use. */
static uint32_t fx_frame;

/* A configured colour as frame_draw_*() paint it, for the frame being
 * drawn (paint_of()) and the row being drawn (paint_row()). */
struct paint {
	enum hub75_effect effect;
	/* EFFECT_NONE, "cycle" and "fade": the colour every pixel gets this
	 * frame, spread(). */
	uint64_t uniform;
	/* Every other effect: its colours for this frame, spread() - 256
	 * along the palette (looked up by hue or heat), or for "sparkle"
	 * each palette colour at SPARKLE_STEPS levels. Built by paint_of(),
	 * so a pixel costs one lookup, not a blend. */
	const uint64_t *lut;
	/* How far the effect has moved: fx_frame * speed. */
	uint32_t t;
	/* The colours it picks from - see pick() - and its size: pattern
	 * coordinates are pixel coordinates times zoom / 256. */
	const uint8_t *pal;
	uint8_t n;
	uint16_t zoom;
	/* Per-row terms, set by paint_row(): the row in pattern
	 * coordinates, and whatever else the effect needs once per row. */
	int sy;
	int row;
};

/* Palette colours `c0` -> `c1` (3-bit, each channel off or full), `f` of
 * 256 of the way, at `level` of 128 brightness - blended per channel in
 * 8 bits, then through gamma to the panel's 4, so the steps in between
 * look even. */
static inline uint32_t blend(uint8_t c0, uint8_t c1, uint8_t f, uint8_t level)
{
	uint8_t out[3];

	for (int ch = 0; ch < 3; ch++) {
		int a = (c0 >> ch) & 1 ? 255 : 0;
		int b = (c1 >> ch) & 1 ? 255 : 0;
		int v = (a * (256 - f) + b * f) >> 8;

		out[ch] = gamma8((v * level) >> 7);
	}
	return RGB24(out[0], out[1], out[2]);
}

/* A 0-255 value going once round - a hue - to the palette as a closed
 * loop of colour stops: the last one blends back into the first. */
static inline uint32_t pick(const struct paint *p, uint8_t v)
{
	unsigned int pos = v * p->n;
	unsigned int i = pos >> 8;

	return blend(p->pal[i], p->pal[(i + 1) % p->n], pos & 0xff, 128);
}

/* A 0-255 value from one end to the other - a heat - to the palette as an
 * open ramp of colour stops, first to last. */
static inline uint32_t pick_ramp(const struct paint *p, uint8_t v)
{
	if (p->n == 1) {
		return blend(p->pal[0], p->pal[0], 0, 128);
	}

	unsigned int pos = v * (p->n - 1);
	unsigned int i = pos >> 8;

	return blend(p->pal[i], p->pal[MIN(i + 1, p->n - 1u)], pos & 0xff, 128);
}

#define SPARKLE_STEPS 32

/* "sparkle"'s twinkle: brightness, of 128, `step` of SPARKLE_STEPS into
 * one - half a sine wave. */
static inline uint8_t sparkle_level(int step)
{
	return (sin8(step * (128 / SPARKLE_STEPS)) * 128) / 127;
}

/* One table per colour field - see struct paint's `lut` - 256 entries
 * each, from hub75_arena. */
#define PAINT_LUT_LEN 256
static uint64_t *paint_luts[2];

/* `c` for the frame about to be drawn, `slot` 0 for the lit pixels' colour,
 * 1 for the unlit ones' (each has its own table). */
static struct paint paint_of(const struct hub75_color *c, int slot)
{
	uint64_t *lut = paint_luts[slot];
	struct paint p = {
		.effect = c->effect,
		.t = fx_frame * c->speed,
		.zoom = c->scale ? (SCALE_DEFAULT << 8) / c->scale : 256,
		.lut = lut,
	};

	effect_palette(c, &p.pal, &p.n);

	uint8_t ph = p.t;

	switch (c->effect) {
	case EFFECT_NONE:
		p.uniform = spread(color_rgb24(c->rgb));
		break;
	case EFFECT_CYCLE:
		/* Blending all the way round the palette. */
		p.uniform = spread(pick(&p, ph));
		break;
	case EFFECT_FADE: {
		/* Each palette colour in turn, fading in from black and out
		 * again - half a sine wave of brightness per colour, 0 right
		 * where the next one takes over. */
		unsigned int pos = ph * p.n;
		uint8_t i = pos >> 8;
		int env = sin8((pos & 0xff) >> 1);

		p.uniform = spread(blend(p.pal[i], p.pal[i], 0, (env * 128) / 127));
		break;
	}
	case EFFECT_SPARKLE:
		for (int i = 0; i < p.n; i++) {
			for (int step = 0; step < SPARKLE_STEPS; step++) {
				lut[i * SPARKLE_STEPS + step] =
					spread(blend(p.pal[i], p.pal[i], 0, sparkle_level(step)));
			}
		}
		break;
	case EFFECT_FIRE:
		for (int v = 0; v < 256; v++) {
			lut[v] = spread(pick_ramp(&p, v));
		}
		break;
	default:
		for (int v = 0; v < 256; v++) {
			lut[v] = spread(pick(&p, v));
		}
		break;
	}
	return p;
}

static inline int zoomed(const struct paint *p, int v)
{
	return (v * p->zoom) >> 8;
}

static bool color_moves(const struct hub75_color *c)
{
	return c->effect != EFFECT_NONE && c->speed != 0;
}

static bool effect_in_use(void)
{
	return color_moves(&active.color) || color_moves(&active.off_color);
}

/* The part of an effect that only depends on the row - once per row. */
static void paint_row(struct paint *p, int y)
{
	uint8_t ph = p->t;

	p->sy = zoomed(p, y);
	switch (p->effect) {
	case EFFECT_PLASMA:
		p->row = sin8(p->sy * 7 - 2 * ph);
		break;
	case EFFECT_FIRE:
		/* Rows up from the bottom - in pixels, it fades by those. */
		p->row = active.height - 1 - y;
		break;
	case EFFECT_RIPPLE:
		p->row = zoomed(p, abs(y - active.height / 2));
		break;
	default:
		p->row = 0;
		break;
	}
}

/* Pixel (x, y) of the display in effect `p`'s colour, spread() - every
 * effect blends between its palette's colours (see pick(), paint_of()),
 * so they're gradients as fine as the panel's colour depth. */
static uint64_t effect_at(const struct paint *p, int x, int y)
{
	uint8_t ph = p->t;
	/* In pattern coordinates - <scale> stretches or shrinks it. */
	int sx = zoomed(p, x);
	int sy = p->sy;

	switch (p->effect) {
	case EFFECT_PLASMA: {
		/* Plasma: a few sine waves over x, y and x + y, each drifting
		 * with the phase, summed and taken as a hue. */
		int v = sin8(sx * 5 + ph) + p->row + sin8((sx + sy) * 4 + ph);

		return p->lut[(uint8_t)((v >> 1) + ph)];
	}
	case EFFECT_STRIPES:
		/* Diagonal bands moving along themselves - a barber pole. */
		return p->lut[(uint8_t)((sx + sy) * 8 - ph)];
	case EFFECT_WAVE:
		/* Horizontal bands, bent up and down by a sine wave running
		 * along them - a flag. */
		return p->lut[(uint8_t)(sy * 8 + (sin8(sx * 8 + ph) >> 2))];
	case EFFECT_SCROLL:
		/* Vertical bands sliding sideways. */
		return p->lut[(uint8_t)(sx * 4 - ph)];
	case EFFECT_CYCLE:
	case EFFECT_FADE:
		/* One colour for every pixel - worked out once per frame, see
		 * paint_of(). */
		return p->uniform;
	case EFFECT_GRADIENT:
		/* A diagonal rainbow, standing still. */
		return p->lut[(uint8_t)((sx + sy) * 4)];
	case EFFECT_RIPPLE: {
		/* Rings moving out from the centre - distance by
		 * max + min * 3/8 (within a few percent of the real one, no
		 * square root). */
		int dx = zoomed(p, abs(x - active.width / 2));
		int dy = p->row;
		int d = MAX(dx, dy) + (MIN(dx, dy) * 3) / 8;

		return p->lut[(uint8_t)(d * 12 - ph)];
	}
	case EFFECT_FIRE: {
		/* Heat: random cells of 2x2 (pattern) pixels rising with time,
		 * fading towards the top, plus a per-pixel flicker - along the
		 * palette by it, coldest first (black, red, yellow, white by
		 * default). */
		int up = p->row;
		uint32_t rise = p->t / 4;
		int cell = hash3(sx >> 1, (zoomed(p, up) + rise) >> 1, 0) & 0xff;
		int flicker = (int)(hash3(x, up, p->t) & 0x3f) - 32;
		int heat = cell * (active.height - up) / active.height + flicker +
			   (up < 2 ? 80 : 0);

		return p->lut[CLAMP(heat, 0, 255)];
	}
	case EFFECT_SPARKLE: {
		/* Every pixel twinkling on its own: fading in and out again
		 * (32 steps of the phase), its own random time to start, in a
		 * new random palette colour every time. */
		uint32_t seed = hash3(sx, sy, 1);
		uint32_t local = p->t + (seed & 0xff);
		uint32_t r = hash3(sx, sy, (local / SPARKLE_STEPS) + 2);
		unsigned int i = (((r >> 8) & 0xff) * p->n) >> 8;

		return p->lut[i * SPARKLE_STEPS + local % SPARKLE_STEPS];
	}
	case EFFECT_NONE:
	default:
		return p->uniform;
	}
}

/* Pixel (x, y) in `p`'s colour, spread(). */
static inline uint64_t paint_at(const struct paint *p, int x, int y)
{
	return p->effect == EFFECT_NONE ? p->uniform : effect_at(p, x, y);
}

/* Per row block (the rows of a panel half sharing one row address, in
 * order) and column of one row of panels (counted from the input's side),
 * its position along that row of panels' part of the address's chain - 0
 * nearest the input connector. Built by chain_build() for the running
 * config, used by frame_locate_row()/frame_row_set(). */
static uint16_t *chain_pos;

/* Each panel takes a panel width's worth of every row block, nearest the
 * input first. Within one, the chain goes through the blocks in pieces of
 * `segment` columns: the first piece of every block in turn (the first
 * block first - chain "n", this DK's panel - or last, "f"), then the
 * second piece of every block, and so on; with `zigzag`, every second
 * piece runs backwards. A piece as wide as the panel (segment 0) gives the
 * plain layout most panels have - one block after the other (the ESP32
 * HUB75 DMA library's FOUR_SCAN_32PX_HIGH is "f"); 1/4 and 1/8 scan panels
 * that interleave their rows in 8- or 16-pixel pieces need "n8", "f16z",
 * ... */
static void chain_build(void)
{
	const int seg_w = active.segment ? active.segment : panel_w;

	for (int b = 0; b < row_blocks; b++) {
		int order = active.far_first ? row_blocks - 1 - b : b;

		for (int x = 0; x < active.width; x++) {
			int panel = x / panel_w;
			int px = x % panel_w;
			int seg = (px / seg_w) * row_blocks + order;
			int off = px % seg_w;

			if (active.zigzag && (seg & 1)) {
				off = seg_w - 1 - off;
			}
			chain_pos[b * active.width + x] =
				panel * panel_w * row_blocks + seg * seg_w + off;
		}
	}
}

/* Where display row `y` (0 at the top, seen from the front) lives in
 * `frame`: pixel x of it is `end` - pos[x, counted from where the chain
 * enters - reversed if `reverse`], in the bits at `shift`.
 *
 * Mirrored by the layout ("r"/"b"), the chain enters the display at the
 * top left, runs through the top row of panels left to right, then the
 * next row - left to right again, or with `serpentine` right to left
 * through panels mounted upside down (so each row's last panel feeds the
 * next one's first with a short ribbon). Every panel's own rows split into
 * halves (R1G1B1, R2G2B2), each half's rows go round-robin over the row
 * addresses (row 9 of a 1/8 scan half is address 1, second block), and
 * `frame` holds each address's whole chain in shift order - the first
 * value shifted ends up furthest along, so position 0 (nearest the input)
 * is its last entry. */
struct frame_row {
	/* The frame entry of chain position 0 (nearest the input), and the
	 * chain positions before this row of panels' - a pixel's position
	 * is before + pos[x], its entry `end` minus that, unless it's past
	 * drive_len (a panel that isn't driven). */
	int end;
	int before;
	const uint16_t *pos;
	bool reverse;
	int shift;
};

static struct frame_row frame_locate_row(int y)
{
	if (active.flip_y) {
		y = active.height - 1 - y;
	}

	int grid_row = y / panel_h;
	int py = y % panel_h;
	/* An upside-down panel's rows and columns both run the other way -
	 * and with the whole row of them, the order the chain takes them in,
	 * which pos[] reversed covers at once. */
	bool upside_down = active.serpentine && (grid_row & 1);

	if (upside_down) {
		py = panel_h - 1 - py;
	}

	int half_rows = panel_h / 2;
	int row = py % half_rows;
	int addr = row % scan_rows;
	int block = row / scan_rows;
	/* The rows of panels before this one, in chain positions. */
	int before = grid_row * active.width * row_blocks;

	return (struct frame_row){
		.end = addr * drive_len + drive_len - 1,
		.before = before,
		.pos = &chain_pos[block * active.width],
		.reverse = active.flip_x != upside_down,
		.shift = (py / half_rows) * 3,
	};
}

/* Pixel x of the row to `planes` (a colour as spread() makes it) - one
 * byte per bit plane. */
static inline void frame_row_set(const struct frame_row *r, int x, uint64_t planes)
{
	int pos = r->before + r->pos[r->reverse ? active.width - 1 - x : x];

	if (pos >= drive_len) {
		return;
	}

	uint8_t *e = &frame[(r->end - pos) * depth];
	const uint8_t keep = ~(0x07 << r->shift);

	for (int k = 0; k < depth; k++, planes >>= 8) {
		e[k] = (e[k] & keep) | ((planes & 0x07) << r->shift);
	}
}

/* nodem-esp32's DriverView::fallback_pixel(), as display_task.c's
 * build_fallback_page() draws it - for a panel the "nodem" config doesn't
 * map: every other pixel around the edge in the colour, which ones
 * alternating with `phase`, the rest in the off colour. */
static void frame_draw_fallback(int phase)
{
	const int w = active.width;
	const int h = active.height;
	struct paint on = paint_of(&active.color, 0);
	struct paint off = paint_of(&active.off_color, 1);

	for (int y = 0; y < h; y++) {
		struct frame_row r = frame_locate_row(y);

		paint_row(&on, y);
		paint_row(&off, y);
		for (int x = 0; x < w; x++) {
			bool edge = x == 0 || y == 0 || x == w - 1 || y == h - 1;
			bool lit = edge && (x + y + phase) % 2 == 0;

			frame_row_set(&r, x, paint_at(lit ? &on : &off, x, y));
		}
	}
}

/* nodem-rs's framebuffer and the config it's rendered with, as last handed
 * over by hub75_task_submit() - only ever touched under display_fb_lock(),
 * like display_task.c's own copy. */
static const uint8_t *nodem_fb;
static struct nodem_config nodem;
static atomic_t nodem_dirty = ATOMIC_INIT(0);

void hub75_task_submit(const uint8_t *buf, const struct nodem_config *config)
{
	display_fb_lock();
	nodem_fb = buf;
	nodem = *config;
	display_fb_unlock();

	atomic_set(&nodem_dirty, 1);
	k_sem_give(&hub75_wake);
}

/* Per display column, the framebuffer column it shows (-1: outside it,
 * off) - through the mapping's offset and scale, nodem_map_axis(). width
 * entries, from hub75_arena. */
static int16_t *map_cols;

/* FNV-1a over the framebuffer and everything it's drawn through - what
 * frame_draw_nodem() last drew, to skip redrawing an unchanged frame
 * (nodem_task submits every one, ~30 a second, changed or not). Call with
 * display_fb_lock() held. */
static uint32_t nodem_hash(void)
{
	uint32_t h = 2166136261u;
	const uint8_t *p = nodem_fb;
	size_t len = nodem_config_buffer_len(&nodem);

	for (size_t i = 0; i < len; i++) {
		h = (h ^ p[i]) * 16777619u;
	}

	const int32_t geometry[] = {
		nodem.width, nodem.height, nodem.hub75.x, nodem.hub75.y,
		nodem.hub75.scale_x, nodem.hub75.scale_y,
	};

	for (size_t i = 0; i < ARRAY_SIZE(geometry); i++) {
		h = (h ^ (uint32_t)geometry[i]) * 16777619u;
	}
	return h;
}

static uint32_t drawn_hash;

/* The framebuffer, through the "hub75" mapping, into `frame` - lit pixels
 * in the colour, the rest in the off colour - unless it's the same frame
 * as last time (`force`: draw regardless). One panel row at a time under
 * display_fb_lock(), so nodem_task never waits on more than a row's worth
 * of it. False if there's no framebuffer or no mapping to draw it
 * through. */
static bool frame_draw_nodem(bool force)
{
	const int w = active.width;
	const int h = active.height;
	struct paint on = paint_of(&active.color, 0);
	struct paint off = paint_of(&active.off_color, 1);

	display_fb_lock();
	if (!nodem_fb || !nodem.hub75.present) {
		display_fb_unlock();
		return false;
	}

	uint32_t hash = nodem_hash();

	display_fb_unlock();
	if (!force && hash == drawn_hash) {
		return true;
	}
	drawn_hash = hash;

	for (int y = 0; y < h; y++) {
		struct frame_row r = frame_locate_row(y);

		paint_row(&on, y);
		paint_row(&off, y);

		display_fb_lock();

		const struct nodem_mapping *m = &nodem.hub75;

		if (!nodem_fb || !m->present) {
			display_fb_unlock();
			return false;
		}

		int fb_w = nodem.width;
		int fb_h = nodem.height;

		if (y == 0) {
			for (int x = 0; x < w; x++) {
				int fx = nodem_map_axis(x, w, fb_w, m->scale_x, m->x);

				map_cols[x] = fx >= 0 && fx < fb_w ? fx : -1;
			}
		}

		int fy = nodem_map_axis(y, h, fb_h, m->scale_y, m->y);

		if (fy < 0 || fy >= fb_h) {
			for (int x = 0; x < w; x++) {
				frame_row_set(&r, x, paint_at(&off, x, y));
			}
		} else {
			/* Row-major, MSB first, no per-row padding - see
			 * nodem_config_buffer_len(). */
			uint32_t row_bit = (uint32_t)fy * fb_w;

			for (int x = 0; x < w; x++) {
				int fx = map_cols[x];
				bool lit = false;

				if (fx >= 0) {
					uint32_t bit = row_bit + fx;

					lit = nodem_fb[bit >> 3] & (0x80 >> (bit & 7));
				}
				frame_row_set(&r, x, paint_at(lit ? &on : &off, x, y));
			}
		}
		display_fb_unlock();
	}
	return true;
}

/* Steps `from`..`to`-1 of one row into seq_a/seq_b, all with OE `oe` -
 * the ISR's hot loop: one byte, one 4-word lookup and two 2-word stores per
 * column. -O2 for just this (the rest of the build is -Os, which runs out
 * of registers here and reloads from the stack every column), and
 * everything it reads comes in as arguments, kept in registers - the
 * compiler can't tell the stores into the step buffers apart from the
 * globals they'd otherwise come from. */
__attribute__((optimize("O2"))) static void
render_span(uint32_t (*restrict a)[2], uint32_t (*restrict b)[2], const uint8_t *restrict src,
	    size_t stride, size_t from, size_t to, uint32_t oe,
	    const uint32_t (*restrict lut)[4])
{
	for (size_t i = from; i < to; i++) {
		/* One plane's byte out of each frame entry. */
		const uint32_t *l = lut[src[i * stride]];

		a[i][0] = l[0];
		a[i][1] = l[1];
		b[i][0] = l[2];
		b[i][1] = l[3] | oe;
	}
}

/* Shift `seq` of the cycle (see shift_seq) - its row address's data for
 * its bit plane - into seq_a/seq_b pair `buf`, with the OE that lights the
 * shift before it (latched while this one shifts) for that one's plane's
 * share. Runs in the ISR once per shift, so it's table lookups only, in
 * three spans by OE: lit, the one partial step, blanked. Only the data
 * steps: the trailing idle ones never change (see hub75_start()). */
static void render_row(uint8_t buf, uint16_t seq)
{
	size_t n = drive_len;
	uint8_t row = seq / depth;
	uint8_t plane = seq % depth;
	/* The plane lit while this shift is going out. */
	uint8_t lit_plane = (plane + depth - 1) % depth;
	size_t lit = MIN(oe_full_steps[lit_plane], n);
	const uint8_t *src = &frame[row * n * depth + plane];

	render_span(seq_a[buf], seq_b[buf], src, depth, 0, lit, STEP_WORD(0, PWM_LOW), lut_px);
	if (lit < n) {
		render_span(seq_a[buf], seq_b[buf], src, depth, lit, lit + 1,
			    oe_partial[lit_plane], lut_px);
		render_span(seq_a[buf], seq_b[buf], src, depth, lit + 1, n,
			    STEP_WORD(0, PWM_HIGH), lut_px);
	}
}

/* Which PWM sequence holds the data: 1 after the filler (sequence 0) when
 * not every panel is driven, else 0. */
static uint8_t data_seq;

static void shift_start(void)
{
	nrf_pwm_seq_ptr_set(PWM_A, data_seq, (const uint16_t *)seq_a[shift_buf]);
	nrf_pwm_seq_ptr_set(PWM_B, data_seq, (const uint16_t *)seq_b[shift_buf]);
	nrf_pwm_event_clear(PWM_A, NRF_PWM_EVENT_STOPPED);
	nrf_pwm_event_clear(PWM_B, NRF_PWM_EVENT_STOPPED);
	nrf_pwm_task_trigger(PWM_A, NRF_PWM_TASK_SEQSTART0);
	nrf_pwm_task_trigger(PWM_B, NRF_PWM_TASK_SEQSTART0);
}

/* The interrupt can't keep the panel going: switch itself off - so it can
 * never take every cycle there is, which at priority 1 would starve every
 * thread, the command handler included, and with the config persisted, at
 * every boot - and leave the rest to hub75_task(). */
static void hub75_overload_trip(atomic_val_t why)
{
	irq_disable(DT_IRQN(PWM_A_NODE));
	atomic_set(&hub75_overload, why);
	k_sem_give(&hub75_wake);
}

static void hub75_isr(const void *arg)
{
	ARG_UNUSED(arg);

	/* PWM_B was started a few cycles after PWM_A, so it can still be in
	 * its last (idle) period - never more than a tick or two. */
	for (int spin = 0; !nrf_pwm_event_check(PWM_B, NRF_PWM_EVENT_STOPPED); spin++) {
		if (spin >= STOP_SPIN_LIMIT) {
			hub75_overload_trip(OVERLOAD_STUCK);
			return;
		}
	}

	/* Both stopped, so OE is back on its GPIO level - high, blanked -
	 * while the address changes and the new row latches. */
	pins_clear(addr_all_mask);
	pins_set(addr_mask(shift_seq / depth));
	/* Held for a microsecond - two back-to-back writes would make a pulse
	 * of only a few tens of ns, which the wiring's capacitance can flatten
	 * before the panel ever sees it. */
	pins_set(BIT(PIN_LAT));
	k_busy_wait(1);
	pins_clear(BIT(PIN_LAT));

	/* The next row was already rendered into the other pair while this
	 * one was shifting - start it right away, then render the one after
	 * it into the pair just freed up. */
	uint16_t cycle = scan_rows * depth;

	shift_seq = (shift_seq + 1) % cycle;
	shift_buf ^= 1;
	shift_start();
	render_row(shift_buf ^ 1, (shift_seq + 1) % cycle);

	/* Stopped again already: rendering took longer than shifting a row,
	 * so this interrupt is due again the moment it returns - once is a
	 * hiccup (a long higher-priority interrupt), every row means it can't
	 * keep up. */
	if (nrf_pwm_event_check(PWM_A, NRF_PWM_EVENT_STOPPED)) {
		if (++overruns >= OVERRUN_LIMIT) {
			hub75_overload_trip(OVERLOAD_OVERRUN);
		}
	} else {
		overruns = 0;
	}
}

/* With every panel driven, one sequence per shift - the data - stopped by
 * its own end (SEQEND0->STOP). Otherwise two, played back to back once
 * (LOOP 1, LOOPSDONE->STOP): the filler first - it's shifted furthest,
 * into the panels that aren't driven, keeping them blank - then the data,
 * into the ones nearest the input. */
static void pwm_setup(NRF_PWM_Type *pwm, const uint32_t pins[NRF_PWM_CHANNEL_COUNT],
		      uint32_t idle_high, uint16_t top, const uint16_t *values,
		      const uint16_t *filler)
{
	const nrf_pwm_sequence_t seq = {
		.values.p_raw = values,
		.length = (drive_len + IDLE_STEPS) * NRF_PWM_CHANNEL_COUNT,
		.repeats = 0,
		.end_delay = 0,
	};
	const nrf_pwm_sequence_t fill = {
		.values.p_raw = filler,
		.length = NRF_PWM_CHANNEL_COUNT,
		.repeats = chain_len - drive_len - 1,
		.end_delay = 0,
	};

	/* While stopped, a PWM pin falls back to its GPIO output level - so
	 * CLK and the data lines idle low between rows, and the pins in
	 * `idle_high` (OE) high. */
	for (size_t i = 0; i < NRF_PWM_CHANNEL_COUNT; i++) {
		pin_cfg_output(pins[i], idle_high & BIT(pins[i]));
	}

	nrf_pwm_pins_set(pwm, pins);
	nrf_pwm_configure(pwm, NRF_PWM_CLK_16MHz, NRF_PWM_MODE_UP, top);
	nrf_pwm_decoder_set(pwm, NRF_PWM_LOAD_INDIVIDUAL, NRF_PWM_STEP_AUTO);
	if (data_seq == 0) {
		nrf_pwm_sequence_set(pwm, 0, &seq);
		nrf_pwm_loop_set(pwm, 0);
		nrf_pwm_shorts_set(pwm, NRF_PWM_SHORT_SEQEND0_STOP_MASK);
	} else {
		nrf_pwm_sequence_set(pwm, 0, &fill);
		nrf_pwm_sequence_set(pwm, 1, &seq);
		nrf_pwm_loop_set(pwm, 1);
		nrf_pwm_shorts_set(pwm, NRF_PWM_SHORT_LOOPSDONE_STOP_MASK);
	}
	nrf_pwm_int_set(pwm, 0);
	nrf_pwm_enable(pwm);
}

/* What a pin goes back to while the panel is disabled: its reset state (an
 * unconnected input), so the DK's LEDs and buttons are left alone - except
 * OE, held high (a panel whose OE floats can keep showing whatever it last
 * latched), and the heartbeat LED's pin, a plain output again for main.c. */
static void pin_release(uint32_t pin)
{
	if (pin == PIN_OE) {
		pin_cfg_output(PIN_OE, true);
	} else if (pin == HEARTBEAT_PIN) {
		nrf_gpio_cfg_output(pin);
	} else {
		nrf_gpio_cfg_default(pin);
	}
}

static void hub75_free(void)
{
	frame = NULL;
	drive_len = 0;
}

static size_t align8(size_t n)
{
	return (n + 7) & ~(size_t)7;
}

/* Every buffer of the config being started (`active`, its geometry
 * worked out), laid out in hub75_arena - for all its panels if they fit,
 * else for as many as do (the first ones along the chain; the rest stay
 * blank, see pwm_setup()). False if not even one does. */
static bool hub75_alloc(void)
{
	const size_t lut_bytes = 2 * align8(PAINT_LUT_LEN * sizeof(uint64_t));
	const size_t pos_bytes = align8(row_blocks * active.width * sizeof(uint16_t));
	const size_t cols_bytes = align8(active.width * sizeof(int16_t));

	for (drive_panels = total_panels; drive_panels > 0; drive_panels--) {
		drive_len = drive_panels * panel_w * row_blocks;

		size_t seq_bytes = align8((drive_len + IDLE_STEPS) * sizeof(step_t));
		size_t frame_bytes = align8((size_t)scan_rows * drive_len * depth);
		size_t total = lut_bytes + 4 * seq_bytes + pos_bytes + cols_bytes + frame_bytes;
		uint8_t *m = hub75_arena;

		if (total > sizeof(hub75_arena)) {
			continue;
		}

		paint_luts[0] = (uint64_t *)m;
		paint_luts[1] = (uint64_t *)(m + lut_bytes / 2);
		m += lut_bytes;
		for (int i = 0; i < 2; i++) {
			seq_a[i] = (step_t *)m;
			m += seq_bytes;
			seq_b[i] = (step_t *)m;
			m += seq_bytes;
		}
		chain_pos = (uint16_t *)m;
		m += pos_bytes;
		map_cols = (int16_t *)m;
		m += cols_bytes;
		frame = m;
		memset(frame, 0, frame_bytes);
		return true;
	}
	drive_len = 0;
	return false;
}

/* Lets the row being shifted finish, stops there, and releases every pin
 * (see pin_release()) and buffer. */
static void hub75_stop(void)
{
	static const uint32_t unconnected[NRF_PWM_CHANNEL_COUNT] = {
		NRF_PWM_PIN_NOT_CONNECTED, NRF_PWM_PIN_NOT_CONNECTED,
		NRF_PWM_PIN_NOT_CONNECTED, NRF_PWM_PIN_NOT_CONNECTED,
	};

	irq_disable(DT_IRQN(PWM_A_NODE));

	/* The SEQEND->STOP shorts stop both at the end of the current row
	 * without the ISR - at most MAX_SEQ_STEPS periods at CLOCK_MIN_KHZ,
	 * ~2.6ms. */
	for (int i = 0; i < 10 && !(nrf_pwm_event_check(PWM_A, NRF_PWM_EVENT_STOPPED) &&
				    nrf_pwm_event_check(PWM_B, NRF_PWM_EVENT_STOPPED));
	     i++) {
		k_msleep(1);
	}

	nrf_pwm_disable(PWM_A);
	nrf_pwm_disable(PWM_B);
	nrf_pwm_pins_set(PWM_A, unconnected);
	nrf_pwm_pins_set(PWM_B, unconnected);
	/* Nothing left over for the next hub75_start() to trip over: no
	 * "stopped" events (they'd fire its interrupt the moment it's
	 * enabled, before the new config's first row is even started), no
	 * interrupt pending. */
	nrf_pwm_int_set(PWM_A, 0);
	nrf_pwm_event_clear(PWM_A, NRF_PWM_EVENT_STOPPED);
	nrf_pwm_event_clear(PWM_B, NRF_PWM_EVENT_STOPPED);
	NVIC_ClearPendingIRQ(DT_IRQN(PWM_A_NODE));

	for (size_t i = 0; i < NRF_PWM_CHANNEL_COUNT; i++) {
		pin_release(pins_a[i]);
		pin_release(pins_b[i]);
	}
	pin_release(PIN_LAT);
	for (size_t i = 0; i < ARRAY_SIZE(addr_pins); i++) {
		pin_release(addr_pins[i]);
	}

	hub75_free();
	running = false;
}

/* --- chip setup, bit-banged before the PWMs take the pins over --- */

#define DATA_PINS_MASK                                                                     \
	(BIT(PIN_R1) | BIT(PIN_G1) | BIT(PIN_B1) | BIT(PIN_R2) | BIT(PIN_G2) | BIT(PIN_B2))

/* One CLK pulse. A microsecond each way - the same wiring capacitance that
 * swallowed a few-ns LAT pulse (see hub75_isr()) applies here; this only
 * runs once per config, so speed doesn't matter. */
static void bitbang_clock(void)
{
	pins_set(BIT(PIN_CLK));
	k_busy_wait(1);
	pins_clear(BIT(PIN_CLK));
	k_busy_wait(1);
}

/* Shifts `bits` (16, repeated along the whole chain - every chip gets the
 * same) into every data line, with LAT held high for the last
 * `latch_clocks` of them - an FM6126A takes LAT high for that many clocks
 * at the end of a shift as "write the register numbered by the count"
 * rather than a plain latch. */
static void fm6126a_write_reg(const uint8_t bits[16], int latch_clocks)
{
	for (int i = 0; i < chain_len; i++) {
		if (bits[i % 16]) {
			pins_set(DATA_PINS_MASK);
		} else {
			pins_clear(DATA_PINS_MASK);
		}
		if (i >= chain_len - latch_clocks) {
			pins_set(BIT(PIN_LAT));
		}
		k_busy_wait(1);
		bitbang_clock();
	}
	pins_clear(BIT(PIN_LAT) | DATA_PINS_MASK);
	k_busy_wait(1);
}

/* FM6126A/FM6124/ICN2038S: their outputs stay off until register 1 (output
 * current - every chip here at the value everyone uses) and register 2
 * (bit 9: outputs on) are written - the same sequence as the ESP32 HUB75
 * DMA library's fm6124init(), which most panels of these chips are tested
 * against. Then one all-off row, so nothing random shows before the first
 * frame. */
static void chip_init_fm6126a(void)
{
	static const uint8_t reg1[16] = { 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0 };
	static const uint8_t reg2[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0 };

	fm6126a_write_reg(reg1, 11);
	fm6126a_write_reg(reg2, 12);

	for (int i = 0; i < chain_len; i++) {
		bitbang_clock();
	}
	pins_set(BIT(PIN_LAT));
	k_busy_wait(1);
	pins_clear(BIT(PIN_LAT));
}

/* Whatever `active.chip` needs before it shows anything - with every HUB75
 * pin a plain GPIO output, OE high (blanked). */
static void chip_init(void)
{
	for (size_t i = 0; i < NRF_PWM_CHANNEL_COUNT; i++) {
		pin_cfg_output(pins_a[i], false);
		pin_cfg_output(pins_b[i], pins_b[i] == PIN_OE);
	}

	switch (active.chip) {
	case CHIP_FM6126A:
		chip_init_fm6126a();
		break;
	case CHIP_SHIFT:
	default:
		break;
	}
}

/* False if there isn't the memory for even one of its panels - it stays
 * off then, OE high. */
static bool hub75_start(const struct hub75_config *c)
{
	active = *c;
	scan_rows = c->scan;
	panel_w = c->width / c->grid_x;
	panel_h = c->height / c->grid_y;
	row_blocks = panel_h / 2 / c->scan;
	chain_len = c->width * c->grid_y * row_blocks;
	total_panels = c->grid_x * c->grid_y;
	depth = c->depth;

	stat_width = c->width;
	stat_height = c->height;
	stat_panels = total_panels;
	if (!hub75_alloc()) {
		error(TAG, "%uB not enough for one panel - disabled, try a lower depth",
		      HUB75_ARENA_SIZE);
		stat_state = STAT_NO_MEM;
		return false;
	}
	stat_driven = drive_panels;
	stat_state = drive_panels < total_panels ? STAT_LOW_MEM : STAT_OK;
	data_seq = drive_panels < total_panels ? 1 : 0;

	/* Even, so CLK can rise exactly halfway through. */
	uint16_t top = (PWM_BASE_KHZ / c->clock_khz) & ~1;

	pwm_top = top;
	oe_set(c->brightness);

	for (size_t px = 0; px < PX_COMBOS; px++) {
		uint8_t t = px & 0x7;
		uint8_t u = (px >> 3) & 0x7;
#define LVL(rgb, bit) (((rgb) & BIT(bit)) ? PWM_HIGH : PWM_LOW)

		lut_px[px][0] = STEP_WORD(LVL(t, 0), LVL(t, 1));
		lut_px[px][1] = STEP_WORD(LVL(t, 2), top / 2);
		lut_px[px][2] = STEP_WORD(LVL(u, 0), LVL(u, 1));
		lut_px[px][3] = STEP_WORD(LVL(u, 2), 0);
#undef LVL
	}
	for (size_t buf = 0; buf < 2; buf++) {
		for (size_t i = drive_len; i < drive_len + IDLE_STEPS; i++) {
			seq_a[buf][i][0] = STEP_WORD(PWM_LOW, PWM_LOW);
			seq_a[buf][i][1] = STEP_WORD(PWM_LOW, PWM_LOW);
			seq_b[buf][i][0] = STEP_WORD(PWM_LOW, PWM_LOW);
			seq_b[buf][i][1] = STEP_WORD(PWM_LOW, PWM_HIGH);
		}
	}
	/* Data off, CLK pulsing, OE blanked. */
	filler_a[0] = STEP_WORD(PWM_LOW, PWM_LOW);
	filler_a[1] = STEP_WORD(PWM_LOW, top / 2);
	filler_b[0] = STEP_WORD(PWM_LOW, PWM_LOW);
	filler_b[1] = STEP_WORD(PWM_LOW, PWM_HIGH);

	addr_all_mask = 0;
	pin_cfg_output(PIN_LAT, false);
	for (size_t i = 0; BIT(i) < scan_rows; i++) {
		addr_all_mask |= BIT(addr_pins[i]);
		pin_cfg_output(addr_pins[i], false);
	}

	chain_build();
	chip_init();

	/* Blank (all off, see hub75_alloc()) until hub75_task() draws the
	 * first frame into it, right after this. */
	render_row(0, 0);
	render_row(1, 1 % (scan_rows * depth));
	shift_seq = 0;
	shift_buf = 0;

	pwm_setup(PWM_A, pins_a, 0, top, (const uint16_t *)seq_a[0], (const uint16_t *)filler_a);
	pwm_setup(PWM_B, pins_b, BIT(PIN_OE), top, (const uint16_t *)seq_b[0],
		  (const uint16_t *)filler_b);

	overruns = 0;
	atomic_set(&hub75_overload, OVERLOAD_NONE);
	nrf_pwm_event_clear(PWM_A, NRF_PWM_EVENT_STOPPED);
	nrf_pwm_event_clear(PWM_B, NRF_PWM_EVENT_STOPPED);
	nrf_pwm_int_set(PWM_A, NRF_PWM_INT_STOPPED_MASK);
	irq_enable(DT_IRQN(PWM_A_NODE));

	/* Both start triggers back to back, uninterrupted - the ISR relies on
	 * PWM_B never trailing PWM_A by more than a couple of ticks. */
	unsigned int key = irq_lock();

	shift_start();
	irq_unlock(key);

	running = true;
	if (drive_panels < total_panels) {
		warn(TAG, "memory for %u of %u panels only", drive_panels, total_panels);
	}
	/* Every row address once per bit plane, each shift taking its
	 * columns plus the idle steps at the pixel clock. */
	uint32_t refresh_hz = (uint32_t)PWM_BASE_KHZ * 1000 / top /
			      ((chain_len + IDLE_STEPS) * scan_rows * depth);

	info(TAG, "%ux%u, %u/%u panels, 1/%u, %ukHz, %u bit, %uHz", c->width, c->height,
	     drive_panels, total_panels, c->scan, PWM_BASE_KHZ / top, depth, refresh_hz);
	return true;
}

/* Set by hub75_task_hold(): the panel stays off until it is cleared again. */
static atomic_t hub75_hold = ATOMIC_INIT(0);

void hub75_task_hold(bool hold)
{
	if (atomic_set(&hub75_hold, hold) != hold) {
		k_sem_give(&hub75_wake);
	}
}

static void hub75_task(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	IRQ_CONNECT(DT_IRQN(PWM_A_NODE), PWM_A_IRQ_PRIORITY, hub75_isr, NULL, 0);


	/* Blanked until the config says otherwise. */
	pin_cfg_output(PIN_OE, true);

	/* Anything but the current generation, so the first pass sets up. */
	atomic_val_t generation = atomic_get(&hub75_generation) - 1;
	/* What's in `frame`: nodem's framebuffer, or the fallback edge in
	 * which phase (-1: neither - redraw whichever applies). */
	bool showing_nodem = false;
	int fallback_phase = -1;
	int64_t next_fx = 0;
	/* Overload trips (see hub75_isr()): when to restart the panel (0:
	 * not), and how many trips since trip_window. */
	int64_t restart_at = 0;
	int64_t trip_window = 0;
	int trips = 0;
	/* hub75_task_hold() seen: the panel is blanked and not (re)started. */
	bool held = false;

	while (true) {
		bool hold = atomic_get(&hub75_hold);

		if (hold != held) {
			held = hold;
			if (held) {
				if (running) {
					hub75_stop();
				}
				restart_at = 0;
			} else {
				/* Set up again from the config, as after any change of it. */
				atomic_inc(&hub75_generation);
			}
		}

		if (atomic_get(&hub75_generation) != generation && !held) {
			generation = atomic_get(&hub75_generation);

			struct hub75_config config;
			bool enabled = hub75_config_read(&config);

			if (running) {
				hub75_stop();
			}
			if (enabled) {
				hub75_start(&config);
			} else {
				stat_state = STAT_DISABLED;
				info(TAG, "disabled");
			}
			showing_nodem = false;
			fallback_phase = -1;
			atomic_set(&nodem_dirty, 1);
			restart_at = 0;
			trips = 0;
		}

		atomic_val_t overload = atomic_set(&hub75_overload, OVERLOAD_NONE);
		int64_t now = k_uptime_get();

		/* A trip is usually transient - code runs from flash, so every
		 * flash write (a config_store save: any "#" command that
		 * persists something, a page change's "last_page") stalls the
		 * ISR mid-render, which a config with little slack (a long
		 * chain at a high pixel clock) can't absorb. So the panel comes
		 * back by itself after OVERLOAD_RESTART_MS - unless it keeps
		 * tripping (OVERLOAD_TRIPS within OVERLOAD_WINDOW_MS): then it
		 * stays off until the config changes. */
		if (overload != OVERLOAD_NONE && running) {
			const char *why = overload == OVERLOAD_STUCK ? "PWM2 stuck" : "overrun";

			hub75_stop();
			if (now - trip_window > OVERLOAD_WINDOW_MS) {
				trip_window = now;
				trips = 0;
			}
			if (++trips >= OVERLOAD_TRIPS) {
				restart_at = 0;
				stat_state = STAT_OVERLOAD;
				error(TAG, "%s x%d - stopped until the config changes (lower the "
				      "pixel clock)", why, trips);
			} else {
				restart_at = now + OVERLOAD_RESTART_MS;
				warn(TAG, "%s - restarting", why);
			}
		}
		if (!running && restart_at && now >= restart_at) {
			struct hub75_config config = active;

			restart_at = 0;
			hub75_start(&config);
			showing_nodem = false;
			fallback_phase = -1;
			atomic_set(&nodem_dirty, 1);
		}

		/* -1: until woken. */
		int64_t wait_ms = restart_at ? MAX(restart_at - now, 0) : -1;

		if (running) {
			/* Shifting, latching and refreshing all run off the
			 * PWM interrupt - this only redraws `frame`: on a
			 * changed nodem frame, every FX_FRAME_MS while a
			 * colour is an effect that moves, once a second for
			 * the fallback. */
			now = k_uptime_get();
			bool animate = false;

			if (effect_in_use()) {
				if (now >= next_fx) {
					fx_frame++;
					next_fx = now + FX_FRAME_MS;
					animate = true;
				}
				wait_ms = next_fx - now;
			}

			bool dirty = atomic_cas(&nodem_dirty, 1, 0);

			if ((dirty || animate) && frame_draw_nodem(animate || !showing_nodem)) {
				showing_nodem = true;
			} else if (!showing_nodem || !nodem.hub75.present) {
				int phase = (int)((now / MSEC_PER_SEC) % 2);
				int64_t to_phase = MSEC_PER_SEC - now % MSEC_PER_SEC;

				showing_nodem = false;
				if (phase != fallback_phase || animate) {
					frame_draw_fallback(phase);
					fallback_phase = phase;
				}
				wait_ms = wait_ms < 0 ? to_phase : MIN(wait_ms, to_phase);
			}
		}
		k_sem_take(&hub75_wake, wait_ms < 0 ? K_FOREVER : K_MSEC(wait_ms));
	}
}

K_THREAD_DEFINE(hub75_tid, HUB75_STACK_SIZE, hub75_task, NULL, NULL, NULL, HUB75_PRIORITY, 0,
		0);
