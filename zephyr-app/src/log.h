#ifndef LOG_H_
#define LOG_H_

#include <stddef.h>
#include <stdint.h>

#include <zephyr/kernel.h>

/*
 * Unified console log format across the whole firmware: "[TAG] LEVEL
 * message". Plain printk() macros rather than Zephyr's CONFIG_LOG subsystem
 * - this project logs over the DK's console UART (uart1 - see the board
 * overlay's `chosen` override and prj.conf), and the format is
 * fixed/simple enough not to need LOG's runtime filtering, deferred
 * logging, or multiple backends.
 *
 * Each macro compiles to a bare ((void)0) once CONFIG_APP_LOG_LEVEL_* (see
 * zephyr-app/Kconfig, set via the Makefile's LOG_LEVEL switch) drops below
 * it - a real compile-time removal (format string and all), not just a
 * runtime-filtered call that still costs flash either way.
 */
#if defined(CONFIG_APP_LOG_LEVEL_DEBUG)
#define debug(tag, fmt, ...) printk("[%s] DEBUG " fmt "\n", tag, ##__VA_ARGS__)
#else
#define debug(tag, fmt, ...) ((void)0)
#endif

#if defined(CONFIG_APP_LOG_LEVEL_DEBUG) || defined(CONFIG_APP_LOG_LEVEL_INFO)
#define info(tag, fmt, ...) printk("[%s] INFO " fmt "\n", tag, ##__VA_ARGS__)
#else
#define info(tag, fmt, ...) ((void)0)
#endif

#if defined(CONFIG_APP_LOG_LEVEL_DEBUG) || defined(CONFIG_APP_LOG_LEVEL_INFO) || \
	defined(CONFIG_APP_LOG_LEVEL_WARN)
#define warn(tag, fmt, ...) printk("[%s] WARN " fmt "\n", tag, ##__VA_ARGS__)
#else
#define warn(tag, fmt, ...) ((void)0)
#endif

#if !defined(CONFIG_APP_LOG_LEVEL_NONE)
#define error(tag, fmt, ...) printk("[%s] ERROR " fmt "\n", tag, ##__VA_ARGS__)
#else
#define error(tag, fmt, ...) ((void)0)
#endif

/*
 * Logs a byte buffer as `dir [<length>] "<trimmed ascii>" <hex>` at INFO
 * level - the ASCII preview is cut at the first '\r'/'\n' (with a "..."
 * marker) rather than embedding raw line breaks in the log, and any
 * trailing whitespace before that cut is trimmed too. The hex dump covers
 * the whole buffer if it's 16 bytes or fewer, otherwise just its first 8
 * and last 8 bytes (split by a "..." marker) - the two ends of a transfer
 * are usually what a bug shows up in, not the middle.
 */
void log_bytes(const char *tag, const char *dir, const uint8_t *buf, size_t len);

#endif /* LOG_H_ */
