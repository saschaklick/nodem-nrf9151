#ifndef MODEM_TASK_H_
#define MODEM_TASK_H_

#include <stddef.h>
#include <stdint.h>

/*
 * Brings up the nRF9151 modem: modem library init, SIM readiness check
 * (plus CA cert provisioning, which must happen in the same offline window
 * as the SIM check - see modem_task.c), and LTE registration with the
 * default PDN context. That's as far as it goes while idle - a TLS
 * connection is only opened when there's an actual reason to: an "#reg"
 * (over UART, provisioning a registration code - see
 * config_store.h/control.rs) triggers a one-shot cloud registration
 * attempt, closed again immediately once it concludes; once registered
 * (a device id on file), it instead opens the device's persistent
 * websocket channel to the cloud (auth'd with the id/secret from
 * registration) and keeps it alive with a periodic ping, reconnecting if it
 * drops. Either way, nothing here holds a connection open (or spends any
 * bandwidth) with no work to do - PDN itself is kept up while idle since
 * LTE attach alone doesn't cost ongoing bandwidth.
 *
 * Runs the sequence continuously - a failed step is retried, and losing the
 * PDN connection drops back to reconnect it - while a second thread prints
 * a one-line status summary ("init=... sim=... pdn=... tls=... reg=...
 * ws=...") at 5Hz; heartbeat_task.c prints the same summary again, just
 * much less often, as a low-frequency liveness log (matching nodem-esp32's
 * heartbeat_task). Self-starting (K_THREAD_DEFINE).
 *
 * Once connected, the websocket also doubles as a second command transport
 * alongside uart_task.c's UART - modem_ws_read()/modem_ws_write() below are
 * nodem_task.c's nodem_process_ws()'s counterpart to
 * uart_task_read()/uart_task_write(), feeding the same
 * nodem_process_command() so "#..." commands work identically over either.
 */

/* Formats the current step statuses, e.g. "init=ok(0) sim=running(0)
 * pdn=ok(0) tls=pending(0) reg=pending(0) ws=pending(0) fd=-1 wsock=-1",
 * into `buf` (NUL-terminated, truncated to fit buf_len). Returns the length
 * the untruncated string would have been (snprintf semantics). */
size_t modem_status_format(char *buf, size_t buf_len);

/* Formats "#stat"'s modem and cloud lines (control.rs), matching
 * nodem-esp32's command_listener.rs "stat" shape:
 *   "modem,<state>,<apn>,<ip>,<rsrp dBm>,<error>\r\n"
 *   "cloud,<registration>,<connection>,<host>,<device_name>,<error>\r\n"
 * <state> is one of connecting/connected/failed, <registration> one of
 * waiting/registering/registered/failed, <connection> one of
 * disconnected/connecting/connected - the same words nodem-esp32 uses.
 * Fields that aren't known yet are left empty. Queries the modem over AT
 * (AT+CGDCONT?/AT+CESQ), so only call it on demand, not periodically.
 * NUL-terminated, truncated to fit buf_len; returns the length the
 * untruncated string would have been (snprintf semantics). */
size_t modem_stat_format(char *buf, size_t buf_len);

/* The modem's internal temperature in whole degrees C (AT%XTEMP?) - the
 * closest thing to a core temperature the nRF9151 exposes. Returns 0 on
 * success, or a negative errno if it couldn't be read (e.g. the modem
 * library isn't initialized yet). */
int modem_core_temp(int *temp_c);

/* The websocket keepalive ping interval, in seconds - config_store's
 * "ping" (1..3600, set via control.rs's "#ping"), defaulting to 60 (and
 * persisting that default) if unset or out of range. */
uint32_t modem_ws_ping_interval_s(void);

/* Pulls up to `max_len` bytes received over the websocket since the last
 * call (FIFO order), removing them. Returns the number of bytes actually
 * copied into `buf` (0 if none are available, including when no websocket
 * is currently connected). Non-blocking. Safe to call from any thread. */
size_t modem_ws_read(uint8_t *buf, size_t max_len);

/* Queues up to `len` bytes to be sent over the websocket next time
 * websocket_supervise() (modem_task.c) gets a chance - typically within
 * MODEM_WS_RECV_TIMEOUT_MS, and only once a websocket is actually
 * connected; queued bytes just wait otherwise. Returns the number of bytes
 * actually queued, which is less than `len` if the queue doesn't have room
 * for all of it - callers aren't expected to retry the remainder (same as
 * uart_task_write(), a dropped/short write here means the peer misses part
 * of a reply, not that the caller should loop). Safe to call from any
 * thread. */
size_t modem_ws_write(const uint8_t *buf, size_t len);

#endif /* MODEM_TASK_H_ */
