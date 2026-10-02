#include "nodem_config.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include "config_store.h"
#include "log.h"

#define TAG "nodem_cfg"

#define NODEM_KEY      "nodem"
#define NODEM_DEFAULT  "128:64:1,oled:0:0:1:1"
/* config_store's MAX_STR_LEN (127) + NUL. */
#define NODEM_VALUE_LEN 128

/* Same upper bound as nodem-esp32's MAX_DIMENSION - on top of
 * NODEM_FB_MAX_SIZE, which is what actually limits it here. */
#define NODEM_MAX_DIMENSION 1024
#define NODEM_MAX_SCALE     16

static atomic_t generation = ATOMIC_INIT(0);

/* Copies the next `sep`-separated field of `*s` into `buf` with spaces
 * trimmed, advancing `*s` past it and its separator (NULL once there's
 * nothing left). False if there's no field left or it doesn't fit. */
static bool next_field(const char **s, char sep, char *buf, size_t buf_len)
{
	if (*s == NULL) {
		return false;
	}

	const char *start = *s;
	const char *end = strchr(start, sep);
	size_t len = end ? (size_t)(end - start) : strlen(start);

	*s = end ? end + 1 : NULL;

	while (len > 0 && *start == ' ') {
		start++;
		len--;
	}
	while (len > 0 && start[len - 1] == ' ') {
		len--;
	}
	if (len >= buf_len) {
		return false;
	}

	memcpy(buf, start, len);
	buf[len] = '\0';
	return true;
}

/* Parses the next ':'-separated field of `*s` as a whole decimal number
 * within `min`..`max`. */
static bool next_num(const char **s, long min, long max, long *out)
{
	char field[16];
	char *end;

	if (!next_field(s, ':', field, sizeof(field)) || field[0] == '\0') {
		return false;
	}

	long v = strtol(field, &end, 10);

	if (*end != '\0' || v < min || v > max) {
		return false;
	}

	*out = v;
	return true;
}

/* "<driver>:<x>:<y>:<scale_x>:<scale_y>" - nodem-esp32's
 * DeviceMapping::parse(), with "oled" and "hub75" as the drivers. `out`
 * gets the mapping of whichever one it names, false if it's already been
 * mapped. */
static bool parse_device(const char *s, struct nodem_config *c)
{
	char driver[8];
	long x, y, scale_x, scale_y;
	struct nodem_mapping *out;

	if (!next_field(&s, ':', driver, sizeof(driver))) {
		return false;
	}
	if (strcmp(driver, "oled") == 0) {
		out = &c->oled;
	} else if (strcmp(driver, "hub75") == 0) {
		out = &c->hub75;
	} else {
		return false;
	}
	if (out->present) {
		return false;
	}
	if (!next_num(&s, INT16_MIN, INT16_MAX, &x) || !next_num(&s, INT16_MIN, INT16_MAX, &y) ||
	    !next_num(&s, 1, NODEM_MAX_SCALE, &scale_x) ||
	    !next_num(&s, 1, NODEM_MAX_SCALE, &scale_y) || s != NULL) {
		return false;
	}

	*out = (struct nodem_mapping){
		.present = true,
		.x = (int16_t)x,
		.y = (int16_t)y,
		.scale_x = (uint8_t)scale_x,
		.scale_y = (uint8_t)scale_y,
	};
	return true;
}

/* nodem-esp32's NodemConfig::parse(): exactly three geometry fields, 1 bit
 * per pixel, each driver mapped at most once, empty device fields (e.g. a
 * trailing comma) skipped. */
static bool parse(const char *s, struct nodem_config *out)
{
	char entry[NODEM_VALUE_LEN];
	const char *geometry;
	long width, height, bpp;
	struct nodem_config c = { 0 };

	if (!next_field(&s, ',', entry, sizeof(entry))) {
		return false;
	}

	geometry = entry;
	if (!next_num(&geometry, 1, NODEM_MAX_DIMENSION, &width) ||
	    !next_num(&geometry, 1, NODEM_MAX_DIMENSION, &height) ||
	    !next_num(&geometry, 1, 1, &bpp) || geometry != NULL) {
		return false;
	}

	c.width = (uint16_t)width;
	c.height = (uint16_t)height;
	c.bits_per_pixel = (uint8_t)bpp;

	if (nodem_config_buffer_len(&c) > NODEM_FB_MAX_SIZE) {
		return false;
	}

	while (s != NULL) {
		if (!next_field(&s, ',', entry, sizeof(entry))) {
			return false;
		}
		if (entry[0] == '\0') {
			continue;
		}
		if (!parse_device(entry, &c)) {
			return false;
		}
	}

	*out = c;
	return true;
}

static void format_device(char *buf, size_t buf_len, int *n, const char *driver,
			  const struct nodem_mapping *m)
{
	if (m->present && *n > 0 && (size_t)*n < buf_len) {
		*n += snprintf(buf + *n, buf_len - *n, ",%s:%d:%d:%u:%u", driver, m->x, m->y,
			       m->scale_x, m->scale_y);
	}
}

/* nodem-esp32's NodemConfig::to_nvs_string() - drivers in a fixed order,
 * whatever order they were given in. */
static void format(const struct nodem_config *c, char *buf, size_t buf_len)
{
	int n = snprintf(buf, buf_len, "%u:%u:%u", c->width, c->height, c->bits_per_pixel);

	format_device(buf, buf_len, &n, "oled", &c->oled);
	format_device(buf, buf_len, &n, "hub75", &c->hub75);
}

void nodem_config_get(struct nodem_config *out)
{
	char value[NODEM_VALUE_LEN];

	config_get_str(NODEM_KEY, value, sizeof(value), "");

	if (parse(value, out)) {
		return;
	}

	if (value[0] != '\0') {
		warn(TAG, "malformed '%s' in config ('%s'), falling back to default", NODEM_KEY,
		     value);
	}
	if (config_set_str(NODEM_KEY, NODEM_DEFAULT)) {
		error(TAG, "failed to persist default '%s'", NODEM_KEY);
	}

	(void)parse(NODEM_DEFAULT, out);
}

int nodem_config_set(const char *value)
{
	struct nodem_config c;
	char normalized[NODEM_VALUE_LEN];

	while (*value == ' ') {
		value++;
	}
	if (!parse(value[0] != '\0' ? value : NODEM_DEFAULT, &c)) {
		return -EINVAL;
	}

	format(&c, normalized, sizeof(normalized));

	int err = config_set_str(NODEM_KEY, normalized);

	if (!err) {
		atomic_inc(&generation);
	}

	return err;
}

uint32_t nodem_config_generation(void)
{
	return (uint32_t)atomic_get(&generation);
}
