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
#include <zephyr/drivers/gpio.h>

#include "config_store.h"
#include "log.h"

#define TAG "main"

#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

int main(void)
{
	info(TAG, "main() entered");

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
