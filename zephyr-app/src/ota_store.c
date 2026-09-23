#include "ota_store.h"

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/dfu/flash_img.h>
#include <zephyr/dfu/mcuboot.h>
#include <zephyr/storage/flash_map.h>

#include <pm_config.h>

#include "log.h"

#define TAG "ota"

/* MCUboot's image header magic (bootutil's IMAGE_MAGIC), the first word of
 * every image imgtool signs. */
#define OTA_IMAGE_MAGIC 0x96f3b83d

static struct flash_img_context ctx;
static size_t expected_len;

size_t ota_store_capacity(void)
{
	return PM_MCUBOOT_SECONDARY_SIZE;
}

int ota_store_start(size_t len)
{
	if (len > PM_MCUBOOT_SECONDARY_SIZE) {
		error(TAG, "%zu bytes won't fit in the %u byte 'mcuboot_secondary' partition", len,
		      PM_MCUBOOT_SECONDARY_SIZE);
		return -ENOSPC;
	}

	int err = flash_img_init_id(&ctx, PM_MCUBOOT_SECONDARY_ID);

	if (err) {
		error(TAG, "flash_img_init_id failed, err %d", err);
		return err;
	}

	expected_len = len;
	return 0;
}

int ota_store_write(const uint8_t *buf, size_t len)
{
	int err = flash_img_buffered_write(&ctx, buf, len, false);

	if (err) {
		error(TAG, "write at offset %zu failed, err %d", flash_img_bytes_written(&ctx), err);
	}

	return err;
}

int ota_store_finish(void)
{
	int err = flash_img_buffered_write(&ctx, NULL, 0, true);

	if (err) {
		error(TAG, "final flush failed, err %d", err);
		return err;
	}

	size_t written = flash_img_bytes_written(&ctx);

	if (written != expected_len) {
		error(TAG, "wrote %zu of %zu bytes", written, expected_len);
		return -EIO;
	}

	const struct flash_area *fa;
	uint32_t magic = 0;

	err = flash_area_open(PM_MCUBOOT_SECONDARY_ID, &fa);
	if (err) {
		error(TAG, "flash_area_open failed, err %d", err);
		return err;
	}

	err = flash_area_read(fa, 0, &magic, sizeof(magic));
	flash_area_close(fa);

	if (err) {
		error(TAG, "header read failed, err %d", err);
		return err;
	}

	if (magic != OTA_IMAGE_MAGIC) {
		error(TAG, "not an MCUboot image (magic 0x%08x)", magic);
		return -EINVAL;
	}

	err = boot_request_upgrade(BOOT_UPGRADE_TEST);
	if (err) {
		error(TAG, "boot_request_upgrade failed, err %d", err);
		return err;
	}

	info(TAG, "%zu byte image pending - swaps in on next reboot", written);
	return 0;
}
