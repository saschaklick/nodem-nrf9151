# Build and flash the nodem-nrf9151 Zephyr firmware.
#
#   make build        - build nodem-ffi (Rust) + the Zephyr app
#   make flash        - flash the last build (merged TF-M+app image)
#   make reset        - reset the board over the debug probe (no reflash)
#   make logs         - stream console logs (printk) over the DK's UART0
#   make reconfigure  - send commands from reconfigure.cmds to the device
#   make run          - build, flash, then stream logs
#   make clean        - remove build output
#
#   make build-rtt, flash-rtt, logs-rtt, run-rtt
#                     - same, but for the RTT-over-SWD logging variant
#                       instead of the UART0 tty - see the note on `logs`
#                       below for when to reach for this instead
#
# After flashing, power-cycle the board (unplug/replug USB, or the reset
# button) - a debugger-triggered reset alone does not reliably re-boot this
# nRF9151/TF-M setup into the new image.
#
# `flash` uses nrfutil (Nordic's own tool, the documented successor to the
# now-deprecated nrfjprog - see https://docs.nordicsemi.com for
# "nrfjprog_deprecation_note"), not probe-rs: nrfutil's `device write`
# handles UICR writes directly (correct memory-controller setup included),
# where probe-rs's own `write` is RAM-only and its hex-file-based approach
# to writing UICR needed an extra, easy-to-get-wrong erase dance to avoid a
# "page write ... failed, code 104" error (confirmed by testing both against
# real hardware side by side - see git history if the probe-rs version of
# this target is ever needed again). $(NRFUTIL) points at its install
# location directly rather than assuming it's on PATH, since it isn't by
# default here (unlike probe-rs, which lives in ~/.cargo/bin).
#
# `flash` erases the whole chip (flash + UICR) before programming: this
# board/firmware combo re-locks its debug access port on every boot
# (TF-M/APPROTECT), so a plain program step fails with a locked-core error
# every single cycle, not just occasionally. This also means config_store's
# persisted state (device_id/device_secret/reg_code/...) never survives a
# flash either; see `reconfigure` below for getting it back without
# retyping commands over `logs`'s console by hand every time.
#
# `flash` then writes UICR.APPROTECT (0x00FF8000) and UICR.SECUREAPPROTECT
# (0x00FF802C, the TF-M/SPE-specific counterpart) to 0x50FA50FA
# (UICR_APPROTECT_PALL_HwUnprotected - see Nordic's nrf9120_bitfields.h),
# every time, not as a one-off. The nRF9151 is a "hardware and software"
# AP-Protect part (see Nordic's ap_protect.rst): unlike older nRF52 chips,
# an erased/blank UICR does NOT mean "debug port stays open" here - the
# firmware-side check (nrf91_handle_approtect(), MDK's SystemInit(), runs in
# every image: mcuboot, TF-M, and the app) only keeps AP-Protect disabled if
# UICR.APPROTECT reads back as this *specific* magic word, not merely
# "erased" (0xFFFFFFFF, a different, still-protected value to this check).
# Because the erase step above wipes UICR back to blank on *every* flash
# (that's what unlocks the AP from the previous boot's lock in the first
# place), this write has to happen every cycle too, right after, in the
# same flash action - doing it once and skipping it on later flashes would
# just get erased again next time. This is the standard, reversible,
# Nordic-documented mechanism (not a permanent unlock): erasing UICR again
# (which `flash` itself does every time, and a bare `recover` would too)
# wipes it back to protected, same as it always did.
#
# Verified against real hardware: flashed, read back both UICR words to
# confirm 0x50FA50FA landed at both addresses, then reset (RESET_SYSTEM) and
# read again from a fresh nrfutil process afterward - debug access survived
# the reset both times tested.
#
# `reset` triggers a plain ARM system reset (RESET_SYSTEM, via CTRL-AP) of
# whatever's currently flashed - it does not reflash anything. Same caveat
# as the note above about power-cycling after `flash`: this is the same
# kind of debugger-triggered reset that doesn't reliably re-boot into a
# *freshly flashed* image, so use it to restart already-running firmware
# (e.g. after using `reconfigure` to change persisted config), not as a
# substitute for a power-cycle right after `flash`.
#
# `logs` reads plain UART (uart0, the DK's onboard J-Link VCOM at
# /dev/ttyACM0). This UART is for console output only - the process_command
# channel uses a different, dedicated UART (uart1) so log text never
# corrupts that protocol's framing.
#
# `logs-rtt` is the alternative: RTT over the same SWD connection used for
# flashing, instead of a second UART wire. This wasn't viable before the
# UICR.APPROTECT/SECUREAPPROTECT fix above - RTT needs a live, unlocked
# debug port to read the target's RAM ring buffer, which used to be locked
# again after every single boot. It needs its own build (`build-rtt`, into
# a separate build-rtt/ directory, via zephyr-app/rtt.conf) rather than
# working against the same image `logs` does: Zephyr's console drivers each
# install their own printk hook at init, and only one of them ends up
# actually active, so UART and RTT console output aren't both live in the
# same build - `build`/`logs` (UART) and `build-rtt`/`logs-rtt` (RTT) are
# two parallel, independent options, not a single switch. `logs-rtt` uses
# JLinkRTTLogger (part of the SEGGER J-Link tools, not probe-rs or nrfutil -
# neither exposes RTT) - the same mechanism probe-rs's own `attach` command
# used to use for this, minus the probe-rs dependency. Its -Device value
# (NRF9151_XXCA) is what J-Link's own device database actually recognizes
# for this chip - confirmed via `JLinkExe -Device nRF9151_xxAA` (the
# nrfutil/probe-rs-style name used everywhere else in this file), which
# reported that name unknown and fell back to NRF9151_XXCA on its own.
# JLinkRTTLogger is a *logger*, not a terminal viewer: real RTT bytes go into
# whatever file you name, while its own connection banner and periodic
# "Transfer rate: ..." status go to the console it's run from - two separate
# streams by design. Pointing the file at /dev/stdout collides them onto one
# fd, which is what put "Transfer rate" lines into the log. `logs-rtt` names
# a real file (RTT_LOG_FILE) instead and lets the status print to the
# terminal as intended (harmless noise there, not in the log); watch it live
# with `tail -F build-rtt/rtt.log` from a second terminal if wanted, entirely
# separate from this target - `-F`, not `-f`: the file doesn't exist until
# `logs-rtt` has actually been started (and JLinkRTTLogger recreates it fresh
# each run), and plain `-f` errors out instead of waiting on a missing file.
#
# `reconfigure` sends each line of reconfigure.cmds (gitignored - it's
# yours to write, typically holding a real registration code and/or device
# name, not something to commit) as a "#..." command to that
# process_command UART - see scripts/reconfigure.sh for the exact protocol
# and why there's no fixed command sequence baked in.

.PHONY: all build nodem-ffi flash reset logs reconfigure run clean \
	build-rtt flash-rtt logs-rtt run-rtt

NCS_ENV := . $(CURDIR)/ncs_env.sh

BOARD       := nrf9151dk/nrf9151/ns
BUILD_DIR   := build
MERGED_HEX  := $(BUILD_DIR)/merged.hex
ELF         := $(BUILD_DIR)/zephyr-app/zephyr/zephyr.elf
NRFUTIL     := $(HOME)/.nrfutil/bin/nrfutil
FAMILY      := nrf91
CONSOLE_TTY := /dev/ttyACM1
CONSOLE_BAUD := 115200
CMD_TTY      := /dev/ttyACM0
CMD_BAUD     := 115200
RECONFIGURE_CMDS := reconfigure.cmds

# UICR_APPROTECT_PALL_HwUnprotected (nrf9120_bitfields.h) - see the header
# comment above for what this is and why it's rewritten on every flash.
APPROTECT_ADDR       := 0x00FF8000
SECUREAPPROTECT_ADDR := 0x00FF802C
APPROTECT_OPEN_VALUE := 0x50FA50FA

BUILD_RTT_DIR  := build-rtt
MERGED_HEX_RTT := $(BUILD_RTT_DIR)/merged.hex
JLINK_DEVICE   := NRF9151_XXCA
RTT_SPEED_KHZ  := 4000
RTT_LOG_FILE   := $(BUILD_RTT_DIR)/rtt.log

all: run

nodem-ffi:
	cd nodem-ffi && cargo build --release --target thumbv8m.main-none-eabi

build: nodem-ffi
	$(NCS_ENV) && west build -b $(BOARD) -d $(BUILD_DIR) zephyr-app

flash:
	$(NRFUTIL) device erase --all --family $(FAMILY)
	$(NRFUTIL) device program --firmware $(MERGED_HEX) --options chip_erase_mode=ERASE_NONE --family $(FAMILY)
	$(NRFUTIL) device write --address $(APPROTECT_ADDR) --value $(APPROTECT_OPEN_VALUE) --family $(FAMILY)
	$(NRFUTIL) device write --address $(SECUREAPPROTECT_ADDR) --value $(APPROTECT_OPEN_VALUE) --family $(FAMILY)
	@echo
	@echo "Flashed - power-cycle the board now (a debugger reset alone won't boot it)."

reset:
	$(NRFUTIL) device reset --reset-kind RESET_SYSTEM --family $(FAMILY)

logs:
	@echo "⏳ Starting logging from $(CONSOLE_TTY)..."	
	@stty -F $(CONSOLE_TTY) $(CONSOLE_BAUD) raw -echo
	@cat $(CONSOLE_TTY)

reconfigure:
	@echo "⏳ Sending configuration commands..."	
	@./scripts/reconfigure.sh $(RECONFIGURE_CMDS) $(CMD_TTY) $(CMD_BAUD)

run: build flash reset sleep reconfigure logs

build-rtt: nodem-ffi
	$(NCS_ENV) && west build -b $(BOARD) -d $(BUILD_RTT_DIR) zephyr-app -- -DEXTRA_CONF_FILE=rtt.conf

flash-rtt:
	$(NRFUTIL) device erase --all --family $(FAMILY)
	$(NRFUTIL) device program --firmware $(MERGED_HEX_RTT) --options chip_erase_mode=ERASE_NONE --family $(FAMILY)
	$(NRFUTIL) device write --address $(APPROTECT_ADDR) --value $(APPROTECT_OPEN_VALUE) --family $(FAMILY)
	$(NRFUTIL) device write --address $(SECUREAPPROTECT_ADDR) --value $(APPROTECT_OPEN_VALUE) --family $(FAMILY)
	@echo
	@echo "Flashed (RTT build) - power-cycle the board now (a debugger reset alone won't boot it)."

logs-rtt:
	JLinkRTTLogger -Device $(JLINK_DEVICE) -If SWD -Speed $(RTT_SPEED_KHZ) -RTTChannel 0 $(RTT_LOG_FILE) &
	sleep 2
	tail -f build-rtt/rtt.log

run-rtt: build-rtt flash-rtt sleep reconfigure logs-rtt

sleep:
	@echo "⏳ Sleeping.."
	@sleep 2

clean:
	rm -rf $(BUILD_DIR) $(BUILD_RTT_DIR)
	cd nodem-ffi && cargo clean
