#ifndef HEARTBEAT_TASK_H_
#define HEARTBEAT_TASK_H_

/*
 * Matches nodem-esp32's heartbeat.rs: a low-frequency (5s) liveness log
 * distinct from modem_task.c's own 5Hz status line - this is meant for a
 * human periodically glancing at the console, not the fast, complete status
 * dump. Currently just modem_status_format()'s summary; nodem-esp32's own
 * heartbeat_task also logs Wi-Fi status, which this project has no
 * equivalent of. Self-starting (K_THREAD_DEFINE) - no public API needed
 * beyond this file existing in the build.
 *
 * Also owns confirming a pending OTA swap permanent after
 * OTA_CONFIRM_UPTIME_MS of uptime (see ota_confirm_if_due() in the .c file)
 * - piggybacking on this task's own periodic wake rather than a separate
 * thread, since "has this task been looping normally" is itself a
 * reasonable-enough stability signal for a single-purpose device like this.
 */

/* Shared with main.c's boot-time log and modem_task.c's status message, so
 * both mention the same number ota_confirm_if_due() actually uses rather
 * than a copy that could drift out of sync with it. */
#define HEARTBEAT_OTA_CONFIRM_SECONDS 60

#endif /* HEARTBEAT_TASK_H_ */
