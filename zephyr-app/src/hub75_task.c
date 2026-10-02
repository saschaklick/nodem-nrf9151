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

/* The longest shift chain - columns shifted per row address: width times
 * the rows of each half that share one address - a config may need. Every
 * buffer is sized for it; hub75_config_parse() rejects anything longer.
 * That's e.g. a 64x64 or 128x32 panel at 1/8 scan, or 64x32 at 1/4. */
#define MAX_CHAIN_LEN 256

/* Two idle steps (CLK low, OE high) after the data: SEQEND fires once the last
 * value has been *read* from RAM, which can be before the previous one has
 * finished playing, and the SEQEND->STOP short only takes effect at the end
 * of the current period - with two trailing idle steps, every data period
 * has fully played by the time the PWM stops. */
#define IDLE_STEPS    2
#define MAX_SEQ_STEPS (MAX_CHAIN_LEN + IDLE_STEPS)

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
	EFFECT_RAINBOW,
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
	{ "rainbow", EFFECT_RAINBOW, 2 },
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

struct hub75_color {
	enum hub75_effect effect;
	/* Effects only: per-frame step, 1-EFFECT_SPEED_MAX (0 for a static
	 * one). */
	uint8_t speed;
	/* EFFECT_NONE only: 0xRRGGBB. */
	uint32_t rgb;
};

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

		if (v > MAX_CHAIN_LEN) {
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
			return true;
		}
	}

	char field[16];
	long v;

	/* "<scan>:<chain>:<clock_khz>:<chip>", each part optional - c
	 * already holds the default's. */
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
	if (!next_field(&s, ':', field, sizeof(field)) || s != NULL) {
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
	return true;
}

/* "<effect>[:<speed>]" (one of `effects`, speed 1-EFFECT_SPEED_MAX, a
 * static one without), or "#" and up to 6 hex digits, 2 per channel, r then
 * g then b - missing digits are 0, same as nodem-esp32's "#iled" colours
 * ("#f" is 0xf00000). */
static bool parse_color(const char *s, struct hub75_color *out)
{
	if (s[0] != '#') {
		char name[12];
		const char *speed = strchr(s, ':');
		size_t len = speed ? (size_t)(speed - s) : strlen(s);

		if (len >= sizeof(name)) {
			return false;
		}
		memcpy(name, s, len);
		name[len] = '\0';

		for (size_t i = 0; i < ARRAY_SIZE(effects); i++) {
			if (strcmp(name, effects[i].name) != 0) {
				continue;
			}

			long v = effects[i].speed;

			if (speed && (effects[i].speed == 0 ||
				      !parse_num(speed + 1, 1, EFFECT_SPEED_MAX, &v))) {
				return false;
			}
			*out = (struct hub75_color){ .effect = effects[i].effect, .speed = v };
			return true;
		}
		return false;
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
	    (c.segment && panel_w % c.segment != 0) ||
	    (uint32_t)c.width * c.grid_y * (panel_h / 2 / c.scan) > MAX_CHAIN_LEN) {
		return -EINVAL;
	}

	*out = c;
	return 1;
}

/* The inverse of parse_color() - an effect's speed only if it isn't its
 * default. */
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
		if (c->speed == effects[i].speed) {
			snprintf(buf, buf_len, "%s", effects[i].name);
		} else {
			snprintf(buf, buf_len, "%s:%u", effects[i].name, c->speed);
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
	char color[16], off_color[16];

	format_color(&c->color, color, sizeof(color));
	format_color(&c->off_color, off_color, sizeof(off_color));
	snprintf(buf, buf_len, "%u:%u:%c%c:%ux%u%s,%u:%s:%u:%s,%s,%s,%u", c->width, c->height,
		 c->flip_x ? 'r' : 'l', c->flip_y ? 'b' : 't', c->grid_x, c->grid_y,
		 c->serpentine ? "i" : "", c->scan, chain, c->clock_khz, chip_name(c->chip), color,
		 off_color, c->brightness);
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
/* One panel's size. */
static uint16_t panel_w;
static uint16_t panel_h;
static uint32_t addr_all_mask;

/* EasyDMA source for each instance - one 16-bit value per channel per step
 * (Individual decoder mode), kept as two 32-bit words per step - channels
 * 0|1 and 2|3, little-endian - so render_row() writes a whole step in two
 * stores. Two of each, ping-ponged: while one pair is being shifted out,
 * the ISR renders the next row into the other. seq_b's channel 3 is OE:
 * while one row is being shifted in, it gates the previously latched one. */
static uint32_t seq_a[2][MAX_SEQ_STEPS][2];
static uint32_t seq_b[2][MAX_SEQ_STEPS][2];

#define STEP_WORD(lo, hi) ((uint32_t)(lo) | ((uint32_t)(hi) << 16))

/* Per `frame` byte (top colour in bits 0-2, bottom in 3-5 - bit 0 red,
 * bit 1 green, bit 2 blue, each), that step's four words: seq_a's R1|G1
 * and B1|CLK, seq_b's R2|G2 and B2 with an empty high half, where OE goes.
 * One lookup per column is all render_span() does. */
#define PX_COMBOS 64
static uint32_t lut_px[PX_COMBOS][4];

/* OE from the brightness: lit (low) for the first `oe_full_steps` data
 * steps of each row, then `oe_partial` (already in the high half: lit for
 * part of that step, or blanked) for the next one, blanked after. */
static uint16_t oe_full_steps;
static uint32_t oe_partial;
/* PWM ticks per column, for the running config. */
static uint16_t pwm_top;

/* OE for `brightness` (0-HUB75_BRIGHTNESS_MAX) - takes effect from the next
 * row rendered, so it can change while the panel runs (see fade_update()).
 * Both halves at once, under irq_lock(), so render_row() never sees one
 * without the other. */
static void oe_set(uint16_t brightness)
{
	uint32_t row_ticks = (uint32_t)chain_len * pwm_top;
	uint32_t on_ticks = (row_ticks * brightness + HUB75_BRIGHTNESS_MAX / 2) /
			    HUB75_BRIGHTNESS_MAX;
	uint16_t partial = on_ticks % pwm_top;
	unsigned int key = irq_lock();

	oe_full_steps = on_ticks / pwm_top;
	oe_partial = STEP_WORD(0, partial ? partial : PWM_HIGH);
	irq_unlock(key);
}

/* What's on the panel, per row address, in shift order (index 0 shifted
 * first): the top half's colour (R1G1B1) in bits 0-2, the bottom half's
 * (R2G2B2) in bits 3-5 - just what one row address's shift
 * needs, in one place. scan_rows rows of chain_len each; written through
 * frame_locate_row()/frame_row_set(). */
static uint8_t frame[HUB75_MAX_PIXELS / 2];

/* Address of the row currently being shifted in - latched and lit by the
 * next end-of-row interrupt - and which seq_a/seq_b pair it's in. */
static uint8_t shift_row;
static uint8_t shift_buf;

static bool running;

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

/* 0xRRGGBB to the 3-bit colour the panel can show for now: each channel
 * on wherever it isn't 0. */
static uint8_t color_bits(uint32_t rgb)
{
	return ((rgb & 0xff0000) ? BIT(0) : 0) | ((rgb & 0x00ff00) ? BIT(1) : 0) |
	       ((rgb & 0x0000ff) ? BIT(2) : 0);
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

/* The fully saturated colours a 1-bit-per-channel panel can show, around
 * the colour wheel: red, yellow, green, cyan, blue, magenta - a hue byte
 * (0-255, once round) picks one. */
static const uint8_t wheel[6] = { 0x1, 0x3, 0x2, 0x6, 0x4, 0x5 };

static inline uint8_t hue_color(uint8_t hue)
{
	return wheel[(hue * 6) >> 8];
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
	/* EFFECT_NONE: the 3-bit colour. */
	uint8_t bits;
	/* How far the effect has moved: fx_frame * speed. */
	uint32_t t;
	/* Per-row term, set by paint_row(). */
	int row;
};

static struct paint paint_of(const struct hub75_color *c)
{
	return (struct paint){
		.effect = c->effect,
		.bits = c->effect == EFFECT_NONE ? color_bits(c->rgb) : 0,
		.t = fx_frame * c->speed,
	};
}

static bool color_moves(const struct hub75_color *c)
{
	return c->effect != EFFECT_NONE && c->speed != 0;
}

static bool effect_in_use(void)
{
	return color_moves(&active.color) || color_moves(&active.off_color);
}

/* "fade": with one bit per channel there's no colour in between two of
 * the wheel's, so it fades through black instead - the whole panel's
 * brightness (OE) follows half a sine wave over every colour step, down to
 * 0 right where the colour changes. Called every effect frame, after
 * fx_frame moved on; the colour (the lit pixels', or else the unlit ones')
 * whose speed it follows is the first that's "fade". */
static void fade_update(void)
{
	const struct hub75_color *c = active.color.effect == EFFECT_FADE ? &active.color
				      : active.off_color.effect == EFFECT_FADE ? &active.off_color
										: NULL;

	if (!c) {
		return;
	}

	uint8_t ph = fx_frame * c->speed;
	/* Where in its colour step the wheel is, 0-255 - a step is 1/6 of
	 * the 256-step wheel (hue_color()). */
	uint8_t pos = (ph * 6) & 0xff;
	int env = sin8(pos >> 1);

	oe_set((uint32_t)active.brightness * env / 127);
}

/* The part of an effect that only depends on the row - once per row. */
static void paint_row(struct paint *p, int y)
{
	uint8_t ph = p->t;

	switch (p->effect) {
	case EFFECT_RAINBOW:
		p->row = sin8(y * 7 - 2 * ph);
		break;
	case EFFECT_FIRE:
		/* Rows up from the bottom. */
		p->row = active.height - 1 - y;
		break;
	case EFFECT_RIPPLE:
		p->row = abs(y - active.height / 2);
		break;
	default:
		p->row = 0;
		break;
	}
}

/* Pixel (x, y) of the display in effect `p`'s colour, 3 bits - only fully
 * saturated colours (and white, black), one bit per channel for now, so
 * every effect moves as bands of them. */
static uint8_t effect_at(const struct paint *p, int x, int y)
{
	uint8_t ph = p->t;

	switch (p->effect) {
	case EFFECT_RAINBOW: {
		/* Plasma: a few sine waves over x, y and x + y, each drifting
		 * with the phase, summed and taken as a hue. */
		int v = sin8(x * 5 + ph) + p->row + sin8((x + y) * 4 + ph);

		return hue_color((uint8_t)((v >> 1) + ph));
	}
	case EFFECT_STRIPES:
		/* Diagonal bands moving along themselves - a barber pole. */
		return hue_color((uint8_t)((x + y) * 8 - ph));
	case EFFECT_WAVE:
		/* Horizontal bands, bent up and down by a sine wave running
		 * along them - a flag. */
		return hue_color((uint8_t)(y * 8 + (sin8(x * 8 + ph) >> 2)));
	case EFFECT_SCROLL:
		/* Vertical bands sliding sideways. */
		return hue_color((uint8_t)(x * 4 - ph));
	case EFFECT_CYCLE:
	case EFFECT_FADE:
		/* One colour for every pixel, stepping round the wheel - "fade"
		 * also dims the whole panel to black around every step, see
		 * fade_update(). */
		return hue_color(ph);
	case EFFECT_GRADIENT:
		/* A diagonal rainbow, standing still. */
		return hue_color((uint8_t)((x + y) * 4));
	case EFFECT_RIPPLE: {
		/* Rings moving out from the centre - distance by
		 * max + min * 3/8 (within a few percent of the real one, no
		 * square root). */
		int dx = abs(x - active.width / 2);
		int dy = p->row;
		int d = MAX(dx, dy) + (MIN(dx, dy) * 3) / 8;

		return hue_color((uint8_t)(d * 12 - ph));
	}
	case EFFECT_FIRE: {
		/* Heat: random cells of 2x2 pixels rising with time, fading
		 * towards the top, plus a per-pixel flicker - black, red,
		 * yellow, white as it gets hotter. */
		int up = p->row;
		uint32_t rise = p->t / 4;
		int cell = hash3(x >> 1, (up + rise) >> 1, 0) & 0xff;
		int flicker = (int)(hash3(x, up, p->t) & 0x3f) - 32;
		int heat = cell * (active.height - up) / active.height + flicker +
			   (up < 2 ? 80 : 0);

		if (heat > 220) {
			return 0x7;
		}
		if (heat > 150) {
			return 0x3;
		}
		if (heat > 80) {
			return 0x1;
		}
		return 0;
	}
	case EFFECT_SPARKLE: {
		/* Every pixel its own random colour, changing at its own
		 * random times - now and then white. */
		uint32_t seed = hash3(x, y, 1);
		uint32_t epoch = (p->t + (seed & 0xff)) >> 5;
		uint32_t r = hash3(x, y, epoch + 2);

		return (r & 0x7) == 0 ? 0x7 : hue_color(r >> 8);
	}
	case EFFECT_NONE:
	default:
		return p->bits;
	}
}

static inline uint8_t paint_at(const struct paint *p, int x, int y)
{
	return p->effect == EFFECT_NONE ? p->bits : effect_at(p, x, y);
}

/* Per row block (the rows of a panel half sharing one row address, in
 * order) and column of one row of panels (counted from the input's side),
 * its position along that row of panels' part of the address's chain - 0
 * nearest the input connector. Built by chain_build() for the running
 * config, used by frame_locate_row()/frame_row_set(). */
static uint16_t chain_pos[MAX_CHAIN_LEN];

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
	uint8_t *end;
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
		.end = &frame[addr * chain_len + chain_len - 1 - before],
		.pos = &chain_pos[block * active.width],
		.reverse = active.flip_x != upside_down,
		.shift = (py / half_rows) * 3,
	};
}

static inline void frame_row_set(const struct frame_row *r, int x, uint8_t rgb)
{
	uint8_t *px = r->end - r->pos[r->reverse ? active.width - 1 - x : x];

	*px = (*px & ~(0x07 << r->shift)) | ((rgb & 0x07) << r->shift);
}

/* nodem-esp32's DriverView::fallback_pixel(), as display_task.c's
 * build_fallback_page() draws it - for a panel the "nodem" config doesn't
 * map: every other pixel around the edge in the colour, which ones
 * alternating with `phase`, the rest in the off colour. */
static void frame_draw_fallback(int phase)
{
	const int w = active.width;
	const int h = active.height;
	struct paint on = paint_of(&active.color);
	struct paint off = paint_of(&active.off_color);

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

/* Per panel column, the framebuffer column it shows (-1: outside it, off)
 * - through the mapping's offset and scale, nodem_map_axis(). Every column
 * is part of the chain, so there are never more than MAX_CHAIN_LEN. */
static int16_t map_cols[MAX_CHAIN_LEN];

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
	struct paint on = paint_of(&active.color);
	struct paint off = paint_of(&active.off_color);

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
	    size_t from, size_t to, uint32_t oe, const uint32_t (*restrict lut)[4])
{
	for (size_t i = from; i < to; i++) {
		const uint32_t *l = lut[src[i]];

		a[i][0] = l[0];
		a[i][1] = l[1];
		b[i][0] = l[2];
		b[i][1] = l[3] | oe;
	}
}

/* Row address `row` of `frame` into seq_a/seq_b pair `buf` - runs in the
 * ISR once per row shifted, so it's table lookups only, in three spans by
 * OE: lit, the one partial step, blanked. Only the data steps: the
 * trailing idle ones never change (see hub75_start()). */
static void render_row(uint8_t buf, uint8_t row)
{
	size_t n = chain_len;
	size_t lit = MIN(oe_full_steps, n);
	const uint8_t *src = &frame[row * n];

	render_span(seq_a[buf], seq_b[buf], src, 0, lit, STEP_WORD(0, PWM_LOW), lut_px);
	if (lit < n) {
		render_span(seq_a[buf], seq_b[buf], src, lit, lit + 1, oe_partial, lut_px);
		render_span(seq_a[buf], seq_b[buf], src, lit + 1, n, STEP_WORD(0, PWM_HIGH),
			    lut_px);
	}
}

static void shift_start(void)
{
	nrf_pwm_seq_ptr_set(PWM_A, 0, (const uint16_t *)seq_a[shift_buf]);
	nrf_pwm_seq_ptr_set(PWM_B, 0, (const uint16_t *)seq_b[shift_buf]);
	nrf_pwm_event_clear(PWM_A, NRF_PWM_EVENT_STOPPED);
	nrf_pwm_event_clear(PWM_B, NRF_PWM_EVENT_STOPPED);
	nrf_pwm_task_trigger(PWM_A, NRF_PWM_TASK_SEQSTART0);
	nrf_pwm_task_trigger(PWM_B, NRF_PWM_TASK_SEQSTART0);
}

static void hub75_isr(const void *arg)
{
	ARG_UNUSED(arg);

	/* PWM_B was started a few cycles after PWM_A, so it can still be in
	 * its last (idle) period - never more than a tick or two. */
	while (!nrf_pwm_event_check(PWM_B, NRF_PWM_EVENT_STOPPED)) {
	}

	/* Both stopped, so OE is back on its GPIO level - high, blanked -
	 * while the address changes and the new row latches. */
	pins_clear(addr_all_mask);
	pins_set(addr_mask(shift_row));
	/* Held for a microsecond - two back-to-back writes would make a pulse
	 * of only a few tens of ns, which the wiring's capacitance can flatten
	 * before the panel ever sees it. */
	pins_set(BIT(PIN_LAT));
	k_busy_wait(1);
	pins_clear(BIT(PIN_LAT));

	/* The next row was already rendered into the other pair while this
	 * one was shifting - start it right away, then render the one after
	 * it into the pair just freed up. */
	shift_row = (shift_row + 1) % scan_rows;
	shift_buf ^= 1;
	shift_start();
	render_row(shift_buf ^ 1, (shift_row + 1) % scan_rows);
}

static void pwm_setup(NRF_PWM_Type *pwm, const uint32_t pins[NRF_PWM_CHANNEL_COUNT],
		      uint32_t idle_high, uint16_t top, const uint16_t *values)
{
	const nrf_pwm_sequence_t seq = {
		.values.p_raw = values,
		.length = (chain_len + IDLE_STEPS) * NRF_PWM_CHANNEL_COUNT,
		.repeats = 0,
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
	nrf_pwm_sequence_set(pwm, 0, &seq);
	nrf_pwm_loop_set(pwm, 0);
	nrf_pwm_shorts_set(pwm, NRF_PWM_SHORT_SEQEND0_STOP_MASK);
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

/* Lets the row being shifted finish, stops there, and releases every pin
 * (see pin_release()). */
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
	NVIC_ClearPendingIRQ(DT_IRQN(PWM_A_NODE));

	for (size_t i = 0; i < NRF_PWM_CHANNEL_COUNT; i++) {
		pin_release(pins_a[i]);
		pin_release(pins_b[i]);
	}
	pin_release(PIN_LAT);
	for (size_t i = 0; i < ARRAY_SIZE(addr_pins); i++) {
		pin_release(addr_pins[i]);
	}

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

static void hub75_start(const struct hub75_config *c)
{
	active = *c;
	scan_rows = c->scan;
	panel_w = c->width / c->grid_x;
	panel_h = c->height / c->grid_y;
	row_blocks = panel_h / 2 / c->scan;
	chain_len = c->width * c->grid_y * row_blocks;

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
		for (size_t i = chain_len; i < chain_len + IDLE_STEPS; i++) {
			seq_a[buf][i][0] = STEP_WORD(PWM_LOW, PWM_LOW);
			seq_a[buf][i][1] = STEP_WORD(PWM_LOW, PWM_LOW);
			seq_b[buf][i][0] = STEP_WORD(PWM_LOW, PWM_LOW);
			seq_b[buf][i][1] = STEP_WORD(PWM_LOW, PWM_HIGH);
		}
	}

	addr_all_mask = 0;
	pin_cfg_output(PIN_LAT, false);
	for (size_t i = 0; BIT(i) < scan_rows; i++) {
		addr_all_mask |= BIT(addr_pins[i]);
		pin_cfg_output(addr_pins[i], false);
	}

	chain_build();
	chip_init();

	/* Blank (all off) until hub75_task() draws the first frame into it,
	 * right after this. */
	memset(frame, 0, sizeof(frame));
	render_row(0, 0);
	render_row(1, 1 % scan_rows);
	shift_row = 0;
	shift_buf = 0;

	pwm_setup(PWM_A, pins_a, 0, top, (const uint16_t *)seq_a[0]);
	pwm_setup(PWM_B, pins_b, BIT(PIN_OE), top, (const uint16_t *)seq_b[0]);

	nrf_pwm_int_set(PWM_A, NRF_PWM_INT_STOPPED_MASK);
	irq_enable(DT_IRQN(PWM_A_NODE));

	/* Both start triggers back to back, uninterrupted - the ISR relies on
	 * PWM_B never trailing PWM_A by more than a couple of ticks. */
	unsigned int key = irq_lock();

	shift_start();
	irq_unlock(key);

	running = true;
	info(TAG, "running: %ux%u (%ux%u%s panels), 1/%u scan, %u-column chain, %s, %ukHz pixel "
	     "clock, brightness %u/%u", c->width, c->height, c->grid_x, c->grid_y,
	     c->serpentine ? " serpentine" : "", c->scan, chain_len, chip_name(c->chip),
	     PWM_BASE_KHZ / top, c->brightness, HUB75_BRIGHTNESS_MAX);
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

	while (true) {
		if (atomic_get(&hub75_generation) != generation) {
			generation = atomic_get(&hub75_generation);

			struct hub75_config config;
			bool enabled = hub75_config_read(&config);

			if (running) {
				hub75_stop();
			}
			if (enabled) {
				hub75_start(&config);
			} else {
				info(TAG, "disabled");
			}
			showing_nodem = false;
			fallback_phase = -1;
			atomic_set(&nodem_dirty, 1);
		}

		/* -1: until woken. */
		int64_t wait_ms = -1;

		if (running) {
			/* Shifting, latching and refreshing all run off the
			 * PWM interrupt - this only redraws `frame`: on a
			 * changed nodem frame, every FX_FRAME_MS while a
			 * colour is an effect that moves, once a second for
			 * the fallback. */
			int64_t now = k_uptime_get();
			bool animate = false;

			if (effect_in_use()) {
				if (now >= next_fx) {
					fx_frame++;
					next_fx = now + FX_FRAME_MS;
					animate = true;
					fade_update();
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
