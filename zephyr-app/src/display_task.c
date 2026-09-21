/*
 * Hand-rolled SSD1306 driver over plain Zephyr i2c_write() calls, run as its
 * own thread that flushes whatever the latest submitted frame is to the
 * panel. Written after Zephyr's own built-in `ssd1306` display driver
 * produced garbled output on this exact panel for reasons never root-caused
 * - this raw implementation (init sequence and command bytes copied from
 * the proven-working `ssd1306-embassy` driver used by this project's
 * earlier embassy-based firmware) was confirmed correct on real hardware
 * first via a static test pattern before being wired up as an async task.
 */

#include "display_task.h"

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>

#include "log.h"

#define TAG "display"

#define SSD1306_ADDR 0x3C

#define CMD_DISPLAY_OFF         0xAE
#define CMD_DISPLAY_ON          0xAF
#define CMD_SET_CONTRAST        0x81
#define CMD_NORMAL_DISPLAY      0xA6
#define CMD_SET_DISPLAY_CLK_DIV 0xD5
#define CMD_SET_MULTIPLEX       0xA8
#define CMD_SET_DISPLAY_OFFSET  0xD3
#define CMD_SET_START_LINE      0x40
#define CMD_CHARGE_PUMP         0x8D
#define CMD_MEMORY_MODE         0x20
#define CMD_SEG_REMAP           0xA1
#define CMD_COM_SCAN_DEC        0xC8
#define CMD_SET_COM_PINS        0xDA
#define CMD_SET_PRECHARGE       0xD9
#define CMD_SET_VCOM_DETECT     0xDB
#define CMD_ENTIRE_DISPLAY_ON   0xA4
#define CMD_DEACTIVATE_SCROLL   0x2E
#define CMD_SET_COLUMN_ADDR     0x21
#define CMD_SET_PAGE_ADDR       0x22

static int write_cmd(const struct device *i2c, uint8_t cmd)
{
	uint8_t buf[2] = { 0x00, cmd };

	return i2c_write(i2c, buf, sizeof(buf), SSD1306_ADDR);
}

static int ssd1306_init(const struct device *i2c)
{
	int err = 0;

	err |= write_cmd(i2c, CMD_DISPLAY_OFF);

	err |= write_cmd(i2c, CMD_SET_DISPLAY_CLK_DIV);
	err |= write_cmd(i2c, 0x80);

	err |= write_cmd(i2c, CMD_SET_MULTIPLEX);
	err |= write_cmd(i2c, 0x3F); /* 1/64 duty (64 rows) */

	err |= write_cmd(i2c, CMD_SET_DISPLAY_OFFSET);
	err |= write_cmd(i2c, 0x00);

	err |= write_cmd(i2c, CMD_SET_START_LINE);

	err |= write_cmd(i2c, CMD_CHARGE_PUMP);
	err |= write_cmd(i2c, 0x14); /* enable charge pump */

	err |= write_cmd(i2c, CMD_MEMORY_MODE);
	err |= write_cmd(i2c, 0x00); /* horizontal addressing mode */

	err |= write_cmd(i2c, CMD_SEG_REMAP);    /* column 127 -> SEG0 */
	err |= write_cmd(i2c, CMD_COM_SCAN_DEC); /* scan COM[N-1] -> COM0 */

	err |= write_cmd(i2c, CMD_SET_COM_PINS);
	err |= write_cmd(i2c, 0x12);

	err |= write_cmd(i2c, CMD_SET_CONTRAST);
	err |= write_cmd(i2c, 0xCF);

	err |= write_cmd(i2c, CMD_SET_PRECHARGE);
	err |= write_cmd(i2c, 0xF1);

	err |= write_cmd(i2c, CMD_SET_VCOM_DETECT);
	err |= write_cmd(i2c, 0x40);

	err |= write_cmd(i2c, CMD_ENTIRE_DISPLAY_ON);
	err |= write_cmd(i2c, CMD_NORMAL_DISPLAY);
	err |= write_cmd(i2c, CMD_DEACTIVATE_SCROLL);

	err |= write_cmd(i2c, CMD_DISPLAY_ON);

	return err;
}

static int ssd1306_write_buffer(const struct device *i2c, const uint8_t *buf)
{
	int err = 0;

	err |= write_cmd(i2c, CMD_SET_COLUMN_ADDR);
	err |= write_cmd(i2c, 0);
	err |= write_cmd(i2c, DISPLAY_WIDTH - 1);

	err |= write_cmd(i2c, CMD_SET_PAGE_ADDR);
	err |= write_cmd(i2c, 0);
	err |= write_cmd(i2c, DISPLAY_HEIGHT / 8 - 1);

	/* One single-shot DMA transfer for the whole frame: nRF9151's TWIM
	 * peripheral is EasyDMA-based with an 8191-byte max transfer (13-bit
	 * EasyDMA counter), comfortably over our 1+DISPLAY_BUF_SIZE payload,
	 * so there's no hardware reason to chunk this into smaller writes. */
	static uint8_t tx_buf[1 + DISPLAY_BUF_SIZE];

	tx_buf[0] = 0x40;
	memcpy(&tx_buf[1], buf, DISPLAY_BUF_SIZE);
	err |= i2c_write(i2c, tx_buf, sizeof(tx_buf), SSD1306_ADDR);

	return err;
}

static uint8_t display_buf[DISPLAY_BUF_SIZE];
K_MUTEX_DEFINE(display_mutex);
static atomic_t display_dirty = ATOMIC_INIT(0);

void display_task_submit(const uint8_t *buf, size_t len)
{
	if (len != DISPLAY_BUF_SIZE) {
		return;
	}

	k_mutex_lock(&display_mutex, K_FOREVER);
	memcpy(display_buf, buf, DISPLAY_BUF_SIZE);
	k_mutex_unlock(&display_mutex);

	atomic_set(&display_dirty, 1);
}

#define DISPLAY_STACK_SIZE 1024
#define DISPLAY_PRIORITY   5
#define DISPLAY_POLL_MS    20

static void display_task(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	const struct device *i2c = DEVICE_DT_GET(DT_NODELABEL(i2c3));

	if (!device_is_ready(i2c)) {
		error(TAG, "i2c device not ready");
		return;
	}

	int err = ssd1306_init(i2c);

	info(TAG, "ssd1306_init err=%d", err);

	static uint8_t local_copy[DISPLAY_BUF_SIZE];

	while (true) {
		if (atomic_cas(&display_dirty, 1, 0)) {
			k_mutex_lock(&display_mutex, K_FOREVER);
			memcpy(local_copy, display_buf, DISPLAY_BUF_SIZE);
			k_mutex_unlock(&display_mutex);

			err = ssd1306_write_buffer(i2c, local_copy);
			if (err) {
				error(TAG, "ssd1306_write_buffer err=%d", err);
			}
		}
		k_msleep(DISPLAY_POLL_MS);
	}
}

K_THREAD_DEFINE(display_tid, DISPLAY_STACK_SIZE, display_task, NULL, NULL, NULL,
		 DISPLAY_PRIORITY, 0, 0);
