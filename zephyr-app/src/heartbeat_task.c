#include "heartbeat_task.h"

#include <zephyr/dfu/mcuboot.h>
#include <zephyr/kernel.h>

#include "modem_task.h"
#include "nodem_ffi.h"
#include "log.h"

#define TAG "heartbeat"

#define HEARTBEAT_PERIOD_MS (5 * MSEC_PER_SEC)

/* How long a freshly-swapped-in OTA image (see the Makefile's `flash-ota
 * SLOT=1`) runs before this confirms it permanent. MCUboot boots a pending
 * ("test") image only once - if it's never confirmed before the next
 * reset, MCUboot reverts back to the previous image on its own. 60s is
 * comfortably past modem_task's own connection/registration steps, so
 * "stable" here really means "got at least one full connection cycle in",
 * not just "didn't crash immediately". */
#define OTA_CONFIRM_UPTIME_MS (HEARTBEAT_OTA_CONFIRM_SECONDS * MSEC_PER_SEC)

/* Runs at most once per boot (guarded by ota_confirmed, not just by the
 * uptime check re-passing every loop once true) - boot_write_img_confirmed()
 * is a flash write, not something to repeat every 5s once already done. */
static void ota_confirm_if_due(void)
{
	static bool ota_confirmed;

	if (ota_confirmed || k_uptime_get() < OTA_CONFIRM_UPTIME_MS) {
		return;
	}

	ota_confirmed = true;

	if (boot_is_img_confirmed()) {
		return;
	}

	int err = boot_write_img_confirmed();

	if (err) {
		error(TAG, "boot_write_img_confirmed failed, err %d", err);
	} else {
		info(TAG, "image confirmed permanent - rollback disabled");
	}
}

static void heartbeat_task(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	char buf[224];

	while (true) {
		modem_status_format(buf, sizeof(buf));
		info(TAG, "%s", buf);

		size_t used, peak, size, failed;

		nodem_heap_stats(&used, &peak, &size, &failed);
		info(TAG, "rust heap used=%u peak=%u size=%u failed=%u", (unsigned)used,
		     (unsigned)peak, (unsigned)size, (unsigned)failed);

		ota_confirm_if_due();

		k_msleep(HEARTBEAT_PERIOD_MS);
	}
}

#define HEARTBEAT_STACK_SIZE 1024
#define HEARTBEAT_PRIORITY   7

K_THREAD_DEFINE(heartbeat_tid, HEARTBEAT_STACK_SIZE, heartbeat_task, NULL, NULL, NULL,
		 HEARTBEAT_PRIORITY, 0, 0);
