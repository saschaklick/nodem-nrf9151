#ifndef CONFIG_STORE_H_
#define CONFIG_STORE_H_

#include <stddef.h>
#include <stdint.h>

/*
 * Small flash-backed key/value store (Zephyr Settings on top of NVS, in the
 * "settings_storage" flash partition partition manager reserves once
 * CONFIG_SETTINGS_NVS is enabled - see prj.conf) for the handful of small
 * strings and numbers this firmware needs to persist across reboots (e.g. a
 * provisioned APN override, a retry count).
 *
 * All persisted keys are loaded into a small RAM cache at config_store_init()
 * (via settings_load()), so config_get_str()/config_get_i32() during normal
 * operation are just a RAM lookup, not a flash read. config_set_str()/
 * config_set_i32() update the cache and write through to flash immediately -
 * call them only when a value actually changes, not on a hot path.
 */

/* Must be called once at startup, before any other thread reads config
 * (main() does this before starting anything else). */
int config_store_init(void);

/* Copies the string stored under `key` into `buf` (always NUL-terminated,
 * truncated to fit `buf_len` if necessary), or `default_val` if `key` has
 * never been set. */
void config_get_str(const char *key, char *buf, size_t buf_len, const char *default_val);

/* Persists `val` under `key`. Returns 0 on success, a negative errno
 * otherwise. */
int config_set_str(const char *key, const char *val);

/* Returns the number stored under `key`, or `default_val` if it has never
 * been set. */
int32_t config_get_i32(const char *key, int32_t default_val);

/* Persists `val` under `key`. Returns 0 on success, a negative errno
 * otherwise. */
int config_set_i32(const char *key, int32_t val);

/* Formats every stored entry as "key=value\r\n" (strings as-is, numbers as
 * decimal) into `buf`, stopping (without emitting a partial line) once an
 * entry wouldn't fit rather than overflowing - see control.rs's "#nvs".
 * Returns the number of bytes written; `buf` is not NUL-terminated. */
size_t config_list(char *buf, size_t buf_len);

#endif /* CONFIG_STORE_H_ */
