#ifndef NODEM_TASK_H_
#define NODEM_TASK_H_

/*
 * Renders nodem-rs's DOM runtime and submits each frame to display_task via
 * display_task_submit(). Self-starting (K_THREAD_DEFINE). Between renders,
 * drains and processes UART/websocket command traffic (including "pkg"
 * uploads) as soon as it arrives rather than only once per render frame -
 * see nodem_task_notify_rx() below.
 */

/* Wakes nodem_task from whatever wait it's currently in (up to the next
 * render frame's deadline) so it drains/processes new UART or websocket
 * bytes immediately instead of waiting out the rest of that deadline first.
 * Call after queuing bytes those transports can read - uart_isr()
 * (uart_task.c) and websocket_supervise() (modem_task.c) are the two
 * current callers. Safe to call from any thread, including an ISR. */
void nodem_task_notify_rx(void);

#endif /* NODEM_TASK_H_ */
