/*
 * Dedicated command UART (uart0, the DK's onboard J-Link VCOM0 - separate
 * USB CDC-ACM interface from the console, which now lives on uart1/VCOM1,
 * see the board overlay's `chosen` override and log.h/prj.conf). 115200
 * baud. Pure process_command protocol traffic only, never console/log
 * text, so nothing can corrupt this protocol's framing.
 *
 * RX is interrupt-driven rather than polled: nRF's UARTE has essentially no
 * RX FIFO depth, so a polling loop checking only every few milliseconds
 * would drop bytes at any real baud rate (this project already hit exactly
 * this problem once before, under the earlier embassy-based firmware,
 * which needed a DMA-backed buffered UART for the same reason). The ISR
 * drains the UART's FIFO into a ring buffer as bytes arrive; nodem_task
 * pulls from it once per render cycle via uart_task_read().
 */

#include "uart_task.h"

#include <errno.h>
#include <stdbool.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>

#include "log.h"
#include "nodem_task.h"

#define TAG "uart"

#define UART_NODE DT_NODELABEL(uart0)
static const struct device *const uart_dev = DEVICE_DT_GET(UART_NODE);

RING_BUF_DECLARE(uart_rx_ring, 1024);

static void uart_isr(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	bool queued = false;

	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (!uart_irq_rx_ready(dev)) {
			continue;
		}

		uint8_t byte;

		while (uart_fifo_read(dev, &byte, 1) == 1) {
			if (ring_buf_put(&uart_rx_ring, &byte, 1) != 1) {
				/* Ring buffer full - drop the byte rather than
				 * block the ISR; nodem_task isn't keeping up. */
				break;
			}
			queued = true;
		}
	}

	if (queued) {
		/* Wakes nodem_task immediately rather than leaving this byte
		 * to wait for its next render deadline - see
		 * nodem_task_notify_rx()'s doc comment. k_sem_give() is
		 * ISR-safe. */
		nodem_task_notify_rx();
	}
}

size_t uart_task_read(uint8_t *buf, size_t max_len)
{
	return ring_buf_get(&uart_rx_ring, buf, max_len);
}

void uart_task_write(const uint8_t *buf, size_t len)
{
	for (size_t i = 0; i < len; i++) {
		uart_poll_out(uart_dev, buf[i]);
	}
}

static int uart_task_init(void)
{
	if (!device_is_ready(uart_dev)) {
		error(TAG, "device not ready");
		return -ENODEV;
	}

	/* No uart_configure() call - baudrate (115200) comes from
	 * `current-speed` in the devicetree (nrf9151dk_nrf9151_common.dtsi's
	 * arduino_serial node), and 8N1/no-flow-control are already Zephyr's
	 * defaults. uart_configure() would need CONFIG_UART_USE_RUNTIME_CONFIGURE
	 * (not enabled) to do anything but return -ENOTSUP. */

	uart_irq_callback_user_data_set(uart_dev, uart_isr, NULL);
	uart_irq_rx_enable(uart_dev);

	info(TAG, "ready");
	return 0;
}

SYS_INIT(uart_task_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
