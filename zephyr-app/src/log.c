#include "log.h"

#include <ctype.h>
#include <stdbool.h>
#include <stdio.h>

void log_bytes(const char *tag, const char *dir, const uint8_t *buf, size_t len)
{
	if (len == 0) {
		return;
	}

	/* Up to 16 "xx "-formatted bytes (8 + 8, split by a "... " gap marker
	 * once len > 16) plus a NUL. */
	char hex[3 * 16 + 4 + 1];
	size_t hex_len = 0;

	if (len <= 16) {
		for (size_t i = 0; i < len; i++) {
			hex_len += snprintf(&hex[hex_len], sizeof(hex) - hex_len, "%02x ", buf[i]);
		}
	} else {
		for (size_t i = 0; i < 8; i++) {
			hex_len += snprintf(&hex[hex_len], sizeof(hex) - hex_len, "%02x ", buf[i]);
		}
		hex_len += snprintf(&hex[hex_len], sizeof(hex) - hex_len, "... ");
		for (size_t i = len - 8; i < len; i++) {
			hex_len += snprintf(&hex[hex_len], sizeof(hex) - hex_len, "%02x ", buf[i]);
		}
	}

	size_t trimmed = len;
	bool truncated = false;

	for (size_t i = 0; i < len; i++) {
		if (buf[i] < 0x20 || buf[i] > 127) {
			trimmed = i;
			truncated = len > i + 2;
			break;
		}
	}	

	info(tag, "%s [%u] \"%.*s%s\" %s", dir, (unsigned)len, (int)trimmed, buf,
	     truncated ? "..." : "", hex);
}
