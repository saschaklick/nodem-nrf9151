#ifndef UART_TASK_H_
#define UART_TASK_H_

#include <stddef.h>
#include <stdint.h>

/*
 * Owns the dedicated process_command UART (uart1, 115200 baud) - interrupt
 * driven RX into a ring buffer, drained by nodem_task before each render.
 */

/* Pulls up to `max_len` received bytes out of the RX ring buffer (FIFO
 * order), removing them. Returns the number of bytes actually copied into
 * `buf` (0 if nothing is available). Non-blocking. */
size_t uart_task_read(uint8_t *buf, size_t max_len);

/* Writes `len` bytes out over the same UART. Blocking (uart_poll_out() per
 * byte) - fine for the small, infrequent command responses this carries. */
void uart_task_write(const uint8_t *buf, size_t len);

#endif /* UART_TASK_H_ */
