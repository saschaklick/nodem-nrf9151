#ifndef NODEM_LISTENER_H_
#define NODEM_LISTENER_H_

/*
 * Whole-device hardware actions invoked from Rust (nodem-ffi's command
 * listener, see control.rs) via nodem_ffi.h's extern "C" declarations - kept
 * separate from modem_task.c (LTE/TLS/cloud-registration-specific) since
 * "control all aspects of NRF hardware" is meant to grow here as more
 * actions get wired in. Currently just a reset.
 */

/* Reboots the device (SYS_REBOOT_COLD). Does not return. Callers must make
 * sure any pending response has already been flushed out (e.g. over UART)
 * before calling this - see ZephyrControl::take_restart() in control.rs and
 * its caller in nodem_task.c. */
void nodem_hw_reset(void);

#endif /* NODEM_LISTENER_H_ */
