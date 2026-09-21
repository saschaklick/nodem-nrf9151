#include "pkg_store.h"

#include <string.h>
#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>

#include <pm_config.h>

#include "log.h"

#define TAG "pkg"

static size_t align_up(size_t val, size_t align)
{
	return (val + align - 1) / align * align;
}

size_t pkg_store_capacity(void)
{
	return PM_PKG_SIZE;
}

int pkg_store_erase(size_t len)
{
	const struct flash_area *fa;
	int err = flash_area_open(PM_PKG_ID, &fa);

	if (err) {
		error(TAG, "flash_area_open failed, err %d", err);
		return err;
	}

	struct flash_pages_info info;
	size_t erase_block = flash_get_page_info_by_offs(fa->fa_dev, fa->fa_off, &info) == 0
				      ? info.size
				      : 4096;

	size_t erase_len = align_up(len, erase_block);

	if (erase_len > fa->fa_size) {
		error(TAG, "%zu bytes won't fit in the %u byte 'pkg' partition", len, fa->fa_size);
		flash_area_close(fa);
		return -ENOSPC;
	}

	err = flash_area_erase(fa, 0, erase_len);
	if (err) {
		error(TAG, "flash_area_erase failed, err %d", err);
	}

	flash_area_close(fa);
	return err;
}

int pkg_store_write(size_t offset, const uint8_t *buf, size_t len)
{
	if (len > PKG_STORE_MAX_CHUNK) {
		error(TAG, "write of %zu bytes exceeds PKG_STORE_MAX_CHUNK (%d)", len,
		      PKG_STORE_MAX_CHUNK);
		return -EINVAL;
	}

	const struct flash_area *fa;
	int err = flash_area_open(PM_PKG_ID, &fa);

	if (err) {
		error(TAG, "flash_area_open failed, err %d", err);
		return err;
	}

	size_t write_block = flash_get_write_block_size(fa->fa_dev);
	size_t aligned_len = align_up(len, write_block);

	if (aligned_len == len) {
		err = flash_area_write(fa, offset, buf, len);
	} else {
		/* Pad the final, sub-write-block chunk with 0xFF - a no-op on
		 * already-erased flash - rather than requiring every caller to
		 * know the device's write-block size. */
		uint8_t padded[PKG_STORE_MAX_CHUNK + 16];

		memcpy(padded, buf, len);
		memset(padded + len, 0xFF, aligned_len - len);
		err = flash_area_write(fa, offset, padded, aligned_len);
	}

	if (err) {
		error(TAG, "flash_area_write at offset %zu failed, err %d", offset, err);
	}

	flash_area_close(fa);
	return err;
}

const uint8_t *pkg_store_data(size_t *len)
{
	*len = PM_PKG_SIZE;
	return (const uint8_t *)PM_PKG_ADDRESS;
}
