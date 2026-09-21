#include "config_store.h"

#include <string.h>
#include <stdio.h>
#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/settings/settings.h>

#include "log.h"

#define TAG "config"

#define MAX_ENTRIES 16
#define MAX_KEY_LEN 15
#define MAX_STR_LEN 127

enum entry_type {
	ENTRY_STR,
	ENTRY_I32,
};

struct entry {
	bool used;
	enum entry_type type;
	char key[MAX_KEY_LEN + 1];
	union {
		char str[MAX_STR_LEN + 1];
		int32_t i32;
	} value;
};

static struct entry entries[MAX_ENTRIES];
static K_MUTEX_DEFINE(entries_lock);

/* Caller must hold entries_lock. Returns the existing entry for `key`/`type`,
 * or - if `create` - a freshly claimed one, or NULL if not found/no room. */
static struct entry *find_entry(const char *key, enum entry_type type, bool create)
{
	struct entry *free_slot = NULL;

	for (int i = 0; i < MAX_ENTRIES; i++) {
		if (entries[i].used && entries[i].type == type &&
		    strcmp(entries[i].key, key) == 0) {
			return &entries[i];
		}
		if (!entries[i].used && !free_slot) {
			free_slot = &entries[i];
		}
	}

	if (!create || !free_slot) {
		return NULL;
	}

	strncpy(free_slot->key, key, MAX_KEY_LEN);
	free_slot->key[MAX_KEY_LEN] = '\0';
	free_slot->type = type;
	free_slot->used = true;

	return free_slot;
}

static int settings_set(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg)
{
	const char *sub_key;

	k_mutex_lock(&entries_lock, K_FOREVER);

	if (settings_name_steq(name, "s", &sub_key) && sub_key) {
		char buf[MAX_STR_LEN + 1] = { 0 };
		ssize_t n = read_cb(cb_arg, buf, sizeof(buf) - 1);
		struct entry *e;

		if (n < 0) {
			k_mutex_unlock(&entries_lock);
			return n;
		}

		e = find_entry(sub_key, ENTRY_STR, true);
		if (!e) {
			k_mutex_unlock(&entries_lock);
			return -ENOMEM;
		}

		memcpy(e->value.str, buf, sizeof(e->value.str));
		k_mutex_unlock(&entries_lock);
		return 0;
	}

	if (settings_name_steq(name, "i", &sub_key) && sub_key) {
		int32_t val;
		ssize_t n = read_cb(cb_arg, &val, sizeof(val));
		struct entry *e;

		if (n != sizeof(val)) {
			k_mutex_unlock(&entries_lock);
			return -EINVAL;
		}

		e = find_entry(sub_key, ENTRY_I32, true);
		if (!e) {
			k_mutex_unlock(&entries_lock);
			return -ENOMEM;
		}

		e->value.i32 = val;
		k_mutex_unlock(&entries_lock);
		return 0;
	}

	k_mutex_unlock(&entries_lock);
	return -ENOENT;
}

SETTINGS_STATIC_HANDLER_DEFINE(config_store, "cfg", NULL, settings_set, NULL, NULL);

void config_get_str(const char *key, char *buf, size_t buf_len, const char *default_val)
{
	const char *src = default_val;

	k_mutex_lock(&entries_lock, K_FOREVER);

	struct entry *e = find_entry(key, ENTRY_STR, false);

	if (e) {
		src = e->value.str;
	}

	strncpy(buf, src, buf_len - 1);
	buf[buf_len - 1] = '\0';

	k_mutex_unlock(&entries_lock);
}

int config_set_str(const char *key, const char *val)
{
	if (strlen(key) > MAX_KEY_LEN || strlen(val) > MAX_STR_LEN) {
		return -EINVAL;
	}

	k_mutex_lock(&entries_lock, K_FOREVER);

	struct entry *e = find_entry(key, ENTRY_STR, true);

	if (!e) {
		k_mutex_unlock(&entries_lock);
		return -ENOMEM;
	}

	strncpy(e->value.str, val, MAX_STR_LEN);
	e->value.str[MAX_STR_LEN] = '\0';

	char full_key[8 + MAX_KEY_LEN + 1];

	snprintf(full_key, sizeof(full_key), "cfg/s/%s", key);

	int err = settings_save_one(full_key, e->value.str, strlen(e->value.str) + 1);

	k_mutex_unlock(&entries_lock);
	return err;
}

int32_t config_get_i32(const char *key, int32_t default_val)
{
	k_mutex_lock(&entries_lock, K_FOREVER);

	struct entry *e = find_entry(key, ENTRY_I32, false);
	int32_t val = e ? e->value.i32 : default_val;

	k_mutex_unlock(&entries_lock);
	return val;
}

int config_set_i32(const char *key, int32_t val)
{
	if (strlen(key) > MAX_KEY_LEN) {
		return -EINVAL;
	}

	k_mutex_lock(&entries_lock, K_FOREVER);

	struct entry *e = find_entry(key, ENTRY_I32, true);

	if (!e) {
		k_mutex_unlock(&entries_lock);
		return -ENOMEM;
	}

	e->value.i32 = val;

	char full_key[8 + MAX_KEY_LEN + 1];

	snprintf(full_key, sizeof(full_key), "cfg/i/%s", key);

	int err = settings_save_one(full_key, &val, sizeof(val));

	k_mutex_unlock(&entries_lock);
	return err;
}

size_t config_list(char *buf, size_t buf_len)
{
	size_t off = 0;

	k_mutex_lock(&entries_lock, K_FOREVER);

	for (int i = 0; i < MAX_ENTRIES; i++) {
		if (!entries[i].used) {
			continue;
		}

		int n = entries[i].type == ENTRY_STR
				? snprintf(&buf[off], buf_len - off, "%s=%s\r\n", entries[i].key,
					   entries[i].value.str)
				: snprintf(&buf[off], buf_len - off, "%s=%d\r\n", entries[i].key,
					   entries[i].value.i32);

		if (n < 0 || (size_t)n >= buf_len - off) {
			break;
		}

		off += (size_t)n;
	}

	k_mutex_unlock(&entries_lock);
	return off;
}

int config_store_init(void)
{
	int err = settings_subsys_init();

	if (err) {
		error(TAG, "settings_subsys_init failed, err %d", err);
		return err;
	}

	err = settings_load();
	if (err) {
		error(TAG, "settings_load failed, err %d", err);
		return err;
	}

	return 0;
}
