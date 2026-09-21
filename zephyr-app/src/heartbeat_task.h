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
 */

#endif /* HEARTBEAT_TASK_H_ */
