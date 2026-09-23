#ifndef OTA_STORE_H_
#define OTA_STORE_H_

#include <stddef.h>
#include <stdint.h>

/*
 * Streams a firmware update - the signed MCUboot image `make ota` produces
 * (nodem_NRF9151-ota.bin) - into the "mcuboot_secondary" slot (see
 * pm_static.yml), written by nodem-ffi's control.rs (IControlLoader, OTA
 * mode) as nodem-rs's "ota" upload protocol streams bytes in. The OTA
 * counterpart of pkg_store.c: same erase-then-forward-only-write shape, but
 * built on Zephyr's flash_img (dfu/flash_img.h) rather than raw flash_area
 * calls, since that already knows how to handle MCUboot's slot trailer.
 *
 * Erasing happens progressively as data arrives (CONFIG_IMG_ERASE_
 * PROGRESSIVELY - see prj.conf), not all up front: erasing a whole 320KB
 * slot in one go takes several seconds, long enough to stall the uploader
 * waiting for its CTR reply.
 */

/* Largest image ota_store can ever hold - the size of the
 * "mcuboot_secondary" partition. */
size_t ota_store_capacity(void);

/* Starts a new upload of `len` bytes, discarding any earlier, unfinished
 * one. Returns 0 on success, a negative errno otherwise (including -ENOSPC
 * if `len` exceeds ota_store_capacity()). */
int ota_store_start(size_t len);

/* Appends `len` bytes from `buf` to the image being uploaded. flash_img
 * buffers internally and flushes whole write blocks as they fill, so any
 * `len` is fine. Returns 0 on success, a negative errno otherwise. */
int ota_store_write(const uint8_t *buf, size_t len);

/* Flushes the last buffered bytes, sanity-checks the MCUboot image header
 * magic at the start of the slot, and marks the slot pending
 * (boot_request_upgrade(BOOT_UPGRADE_TEST)): MCUboot swaps it in on the next
 * reboot as a one-time test, and reverts unless the new image confirms
 * itself (heartbeat_task.c's ota_confirm_if_due()). Does not reboot by
 * itself. Returns 0 on success, a negative errno otherwise. */
int ota_store_finish(void);

#endif /* OTA_STORE_H_ */
