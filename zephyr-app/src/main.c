/*
 * Entry point. The real work happens in display_task.c (owns the SSD1306
 * over I2C), nodem_task.c (renders nodem-rs at ~30Hz and hands frames to
 * display_task), and modem_task.c (LTE/TLS) - all self-start via
 * K_THREAD_DEFINE at kernel init, before main() runs. This initializes the
 * config store first (so those threads never race its first load), then
 * just blinks the LED as a liveness heartbeat.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/dfu/mcuboot.h>
#include <zephyr/drivers/gpio.h>

#include "config_store.h"
#include "heartbeat_task.h"
#include "log.h"

#define TAG "main"

#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

int main(void)
{
	info(TAG, "main() entered");

	/* MCUboot always executes from the primary slot - there's no "running
	 * from slot 1" to report post-swap, the swap physically moves bytes
	 * into slot 0 before this ever runs. What *is* knowable: whether this
	 * boot is a freshly-swapped-in OTA image MCUboot is still only
	 * running as a one-time test (boot_is_img_confirmed() == false, see
	 * the Makefile's `flash-ota SLOT=1` and heartbeat_task.c's own note),
	 * or an already-confirmed image (either the original factory image,
	 * or a previously-confirmed OTA update - the two are indistinguishable
	 * from here once confirmed, since confirming doesn't record *when*). */
	if (boot_is_img_confirmed()) {
		info(TAG, "boot: image confirmed");
	} else {
		info(TAG, "boot: image pending - OTA test swap, reverts if not confirmed within %ds",
		     HEARTBEAT_OTA_CONFIRM_SECONDS);
	}

	int err = config_store_init();

	if (err) {
		error(TAG, "config_store_init failed, err %d", err);
	}

	if (!gpio_is_ready_dt(&led)) {
		error(TAG, "led device not ready");
		return 0;
	}
	gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);

	while (true) {
		gpio_pin_toggle_dt(&led);
		k_msleep(500);
	}

	return 0;
}
