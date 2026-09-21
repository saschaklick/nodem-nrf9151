# Build and flash the nodem-nrf9151 Zephyr firmware.
#
#   make build        - build nodem-ffi (Rust) + the Zephyr app
#   make flash        - flash the last build (merged TF-M+app image)
#   make logs         - stream console logs (printk) over the DK's UART0
#   make reconfigure  - send commands from reconfigure.cmds to the device
#   make run          - build, flash, then stream logs
#   make clean        - remove build output
#
# After flashing, power-cycle the board (unplug/replug USB, or the reset
# button) - a debugger-triggered reset alone does not reliably re-boot this
# nRF9151/TF-M setup into the new image.
#
# `flash` always passes --allow-erase-all: this board/firmware combo
# re-locks its debug access port on every boot (TF-M/APPROTECT), so a plain
# flash fails with "Core 0 is locked" every single cycle, not just
# occasionally. The erase is a full chip wipe (flash + UICR), which is fine
# here since this DK only ever runs our own firmware - but it also means
# config_store's persisted state (device_id/device_secret/reg_code/...)
# never survives a flash either; see `reconfigure` below for getting it
# back without retyping commands over `logs`'s console by hand every time.
#
# `logs` reads plain UART (uart0, the DK's onboard J-Link VCOM at
# /dev/ttyACM0) rather than RTT over SWD - RTT needs an open debug port,
# which is exactly what's locked after every boot per the above, so it can
# never actually capture a running board. UART0 is a separate USB CDC-ACM
# interface, unaffected by the SWD lock. This UART is for console output
# only - the future process_command channel uses a different, dedicated
# UART (uart1) so log text never corrupts that protocol's framing.
#
# `reconfigure` sends each line of reconfigure.cmds (gitignored - it's
# yours to write, typically holding a real registration code and/or device
# name, not something to commit) as a "#..." command to that
# process_command UART - see scripts/reconfigure.sh for the exact protocol
# and why there's no fixed command sequence baked in.

.PHONY: all build nodem-ffi flash logs reconfigure run clean

NCS_ENV := . $(CURDIR)/ncs_env.sh

BOARD       := nrf9151dk/nrf9151/ns
BUILD_DIR   := build
MERGED_HEX  := $(BUILD_DIR)/merged.hex
ELF         := $(BUILD_DIR)/zephyr-app/zephyr/zephyr.elf
CHIP        := nRF9151_xxAA
CONSOLE_TTY := /dev/ttyACM1
CONSOLE_BAUD := 115200
CMD_TTY      := /dev/ttyACM0
CMD_BAUD     := 115200
RECONFIGURE_CMDS := reconfigure.cmds

all: run

nodem-ffi:
	cd nodem-ffi && cargo build --release --target thumbv8m.main-none-eabi

build: nodem-ffi
	$(NCS_ENV) && west build -b $(BOARD) -d $(BUILD_DIR) zephyr-app

flash:
	probe-rs download --binary-format hex --chip $(CHIP) --allow-erase-all $(MERGED_HEX)
	@echo
	@echo "Flashed - power-cycle the board now (a debugger reset alone won't boot it)."

logs:
	stty -F $(CONSOLE_TTY) $(CONSOLE_BAUD) raw -echo
	cat $(CONSOLE_TTY)

reconfigure:
	./scripts/reconfigure.sh $(RECONFIGURE_CMDS) $(CMD_TTY) $(CMD_BAUD)

run: build flash sleep3 reconfigure logs

sleep3:
	sleep 3

clean:
	rm -rf $(BUILD_DIR)
	cd nodem-ffi && cargo clean
