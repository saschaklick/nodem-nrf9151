# Build and flash the nodem-nrf9151 Zephyr firmware.
#
#   make build        - build nodem-ffi (Rust) + the Zephyr app
#   make flash        - flash the last build, keeping config_store/pkg intact
#   make flash INIT=1 - same, but mass-erases first (needed once per locked
#                       or brand-new chip - see its own note)
#   make reset        - reset the board over the debug probe (no reflash)
#   make logs         - stream console logs (printk) over the DK's UART0
#   make reconfigure  - send commands from reconfigure.cmds to the device
#   make run          - build, flash, then stream logs
#   make clean        - remove build output
#
#   LOG=rtt make <build|flash|logs|run>
#                     - same targets, but for the RTT-over-SWD logging
#                       variant instead of the UART0 tty - see the note on
#                       `logs` below for when to reach for this instead.
#                       Builds into a separate build-rtt/ directory (see
#                       BUILD_RTT_DIR below) so switching LOG back and forth
#                       doesn't force a full rebuild each time - only the
#                       target you actually run needs LOG=rtt; `make flash
#                       LOG=rtt` after `make build LOG=rtt` is enough, no
#                       need to repeat it on every single invocation in a
#                       session that's staying in RTT mode, but there's no
#                       persisted default either - it's just $(LOG) per call.
#                       Combines with INIT=1 the same way: `make flash
#                       LOG=rtt INIT=1`.
#
#   LOG_LEVEL=debug|info|warn|error|none make <build|run>
#                     - which of log.h's levels (debug/info/warn/error) get
#                       compiled in at all, default "debug" (everything);
#                       each level includes itself and everything more
#                       severe, "none" drops logging entirely. This is a
#                       genuine compile-time removal (see zephyr-app/Kconfig
#                       and log.h), not a runtime verbosity filter, and
#                       covers nodem-ffi's own log::info!/error! calls too
#                       (see nodem-ffi/Cargo.toml) - real flash savings, not
#                       just a quieter console. Composes with LOG=rtt the
#                       same way INIT=1 does: `make build LOG=rtt
#                       LOG_LEVEL=error`.
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
# Plain `make flash` (INIT unset/0) is the default, everyday one: it uses
# nrfutil's chip_erase_mode=ERASE_RANGES_TOUCHED_BY_FIRMWARE, which only
# erases the flash pages merged.hex actually covers - mcuboot +
# mcuboot_primary (mcuboot_pad+tfm+app) - never touching settings_storage/
# pkg/nonsecure_storage at higher addresses, so config_store's persisted
# state (device_id/device_secret/reg_code/...) and any uploaded "pkg"
# survive an ordinary reflash. It also never touches UICR, so whatever the
# last `INIT=1` flash wrote to UICR.APPROTECT/SECUREAPPROTECT (see below)
# stays in place, keeping the debug port open through this reflash too.
#
# This only works once the AP is already unlocked, though: on a genuinely
# locked or brand-new chip it fails with a locked-core error, because this
# board/firmware combo re-locks its debug access port on every boot
# (TF-M/APPROTECT). That's what `INIT=1` (below) exists to get past - run
# `make flash INIT=1` once for a new/locked chip, then plain `make flash`
# for every reflash after that.
#
# `make flash INIT=1` is the full-recovery version: it mass-erases the whole
# chip (flash + UICR) before programming, which is what actually gets past
# an already-locked debug port (a locked AP still accepts this specific
# mass-erase/recover operation, by hardware design - that's the "recovery"
# mechanism, not a bug). This also wipes config_store's persisted state and
# any "pkg" along with everything else; see `reconfigure` below for getting
# device registration back afterward without retyping commands over `logs`'s
# console by hand every time.
#
# `INIT=1` then writes UICR.APPROTECT (0x00FF8000) and UICR.SECUREAPPROTECT
# (0x00FF802C, the TF-M/SPE-specific counterpart) to 0x50FA50FA
# (UICR_APPROTECT_PALL_HwUnprotected - see Nordic's nrf9120_bitfields.h).
# The nRF9151 is a "hardware and software" AP-Protect part (see Nordic's
# ap_protect.rst): unlike older nRF52 chips, an erased/blank UICR does NOT
# mean "debug port stays open" here - the firmware-side check
# (nrf91_handle_approtect(), MDK's SystemInit(), runs in every image:
# mcuboot, TF-M, and the app) only keeps AP-Protect disabled if
# UICR.APPROTECT reads back as this *specific* magic word, not merely
# "erased" (0xFFFFFFFF, a different, still-protected value to this check).
# Because `INIT=1`'s own mass erase wipes UICR back to blank first, writing
# this magic value back is what actually re-opens the debug port each time
# `INIT=1` runs. This is the standard, reversible, Nordic-documented
# mechanism (not a permanent unlock): erasing UICR again (a bare `recover`,
# or another `INIT=1` flash) wipes it back to protected, same as it always
# did - plain `make flash` (above) relies on this value surviving untouched
# from the last `INIT=1` flash, which is exactly what its narrower erase
# range guarantees.
#
# Verified against real hardware: flashed, read back both UICR words to
# confirm 0x50FA50FA landed at both addresses, then reset (RESET_SYSTEM) and
# read again from a fresh nrfutil process afterward - debug access survived
# the reset both times tested.
#
# `reset` triggers a plain ARM system reset (RESET_SYSTEM, via CTRL-AP) of
# whatever's currently flashed - it does not reflash anything. Same caveat
# as the note above about power-cycling after flashing: this is the same
# kind of debugger-triggered reset that doesn't reliably re-boot into a
# *freshly flashed* image, so use it to restart already-running firmware
# (e.g. after using `reconfigure` to change persisted config), not as a
# substitute for a power-cycle right after flashing.
#
# `logs` reads plain UART (uart0, the DK's onboard J-Link VCOM at
# /dev/ttyACM0). This UART is for console output only - the process_command
# channel uses a different, dedicated UART (uart1) so log text never
# corrupts that protocol's framing.
#
# `LOG=rtt make logs` is the alternative: RTT over the same SWD connection
# used for flashing, instead of a second UART wire. This wasn't viable
# before the UICR.APPROTECT/SECUREAPPROTECT fix above - RTT needs a live,
# unlocked debug port to read the target's RAM ring buffer, which used to be
# locked again after every single boot. It needs its own build (into a
# separate build-rtt/ directory, via zephyr-app/rtt.conf) rather than
# working against the same image plain `logs` does: Zephyr's console drivers
# each install their own printk hook at init, and only one of them ends up
# actually active, so UART and RTT console output aren't both live in the
# same build - this is a genuine compile-time fork (a different Kconfig, and
# thus a different compiled image), not something a runtime switch could
# collapse into one binary; $(LOG) only picks which of the two already-built
# images/commands a given `make` invocation targets. `LOG=rtt make logs`
# uses JLinkRTTLogger (part of the SEGGER J-Link tools, not probe-rs or
# nrfutil - neither exposes RTT) - the same mechanism probe-rs's own
# `attach` command used to use for this, minus the probe-rs dependency. Its
# -Device value (NRF9151_XXCA) is what J-Link's own device database actually
# recognizes for this chip - confirmed via `JLinkExe -Device nRF9151_xxAA`
# (the nrfutil/probe-rs-style name used everywhere else in this file), which
# reported that name unknown and fell back to NRF9151_XXCA on its own.
# JLinkRTTLogger is a *logger*, not a terminal viewer: real RTT bytes go into
# whatever file you name, while its own connection banner and periodic
# "Transfer rate: ..." status go to the console it's run from - two separate
# streams by design. Pointing the file at /dev/stdout collides them onto one
# fd, which is what put "Transfer rate" lines into the log. `logs` (in RTT
# mode) instead names a real file (RTT_LOG_FILE) and lets the status print to
# the terminal as intended (harmless noise there, not in the log); watch it
# live with `tail -F build-rtt/rtt.log` from a second terminal if wanted,
# entirely separate from this target - `-F`, not `-f`: the file doesn't
# exist until `logs` has actually been started (and JLinkRTTLogger recreates
# it fresh each run), and plain `-f` errors out instead of waiting on a
# missing file.
#
# `reconfigure` sends each line of reconfigure.cmds (gitignored - it's
# yours to write, typically holding a real registration code and/or device
# name, not something to commit) as a "#..." command to that
# process_command UART - see scripts/reconfigure.sh for the exact protocol
# and why there's no fixed command sequence baked in.

.PHONY: all build nodem-ffi flash reset logs reconfigure run clean

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
# comment above for what this is and why it's rewritten on every INIT=1 flash.
APPROTECT_ADDR       := 0x00FF8000
SECUREAPPROTECT_ADDR := 0x00FF802C
APPROTECT_OPEN_VALUE := 0x50FA50FA

BUILD_RTT_DIR  := build-rtt
MERGED_HEX_RTT := $(BUILD_RTT_DIR)/merged.hex
JLINK_DEVICE   := NRF9151_XXCA
RTT_SPEED_KHZ  := 4000
RTT_LOG_FILE   := $(BUILD_RTT_DIR)/rtt.log

# LOG=uart (default) or LOG=rtt - see the header comment's "LOG=rtt make
# ..." note. Picks which of the two independently-built images/commands
# `build`/`flash`/`logs` actually target; it's not a build input in itself;
# the RTT image gets its own Kconfig fragment (zephyr-app/rtt.conf) via
# EXTRA_CONF_ARGS below.
LOG ?= uart

# LOG_LEVEL=debug (default)|info|warn|error|none - see the header comment's
# "LOG_LEVEL=... make" note. Maps to one zephyr-app/log-*.conf Kconfig
# fragment (debug needs none - it's APP_LOG_LEVEL's Kconfig default) and one
# nodem-ffi Cargo feature (log-debug/log-info/log-warn/log-error/log-none -
# see nodem-ffi/Cargo.toml), so both sides drop the same levels together.
LOG_LEVEL ?= debug

ifeq ($(LOG_LEVEL),info)
LOG_LEVEL_CONF    := log-info.conf
CARGO_LOG_FEATURE := log-info
else ifeq ($(LOG_LEVEL),warn)
LOG_LEVEL_CONF    := log-warn.conf
CARGO_LOG_FEATURE := log-warn
else ifeq ($(LOG_LEVEL),error)
LOG_LEVEL_CONF    := log-error.conf
CARGO_LOG_FEATURE := log-error
else ifeq ($(LOG_LEVEL),none)
LOG_LEVEL_CONF    := log-none.conf
CARGO_LOG_FEATURE := log-none
else
LOG_LEVEL_CONF    :=
CARGO_LOG_FEATURE := log-debug
endif

# Zephyr's EXTRA_CONF_FILE takes a ';'-separated list, so LOG=rtt's rtt.conf
# and LOG_LEVEL's log-*.conf (either, neither, or both may apply) compose
# into one -DEXTRA_CONF_FILE rather than needing their own separate build
# dirs/targets.
ifeq ($(LOG),rtt)
ACTIVE_BUILD_DIR  := $(BUILD_RTT_DIR)
ACTIVE_MERGED_HEX := $(MERGED_HEX_RTT)
ifeq ($(LOG_LEVEL_CONF),)
CONF_FILES := rtt.conf
else
CONF_FILES := rtt.conf;$(LOG_LEVEL_CONF)
endif
else
ACTIVE_BUILD_DIR  := $(BUILD_DIR)
ACTIVE_MERGED_HEX := $(MERGED_HEX)
CONF_FILES := $(LOG_LEVEL_CONF)
endif

ifeq ($(CONF_FILES),)
EXTRA_CONF_ARGS :=
else
EXTRA_CONF_ARGS := -- -DEXTRA_CONF_FILE="$(CONF_FILES)"
endif

# INIT=0 (default) or INIT=1 - see the header comment's "make flash INIT=1"
# note. Switches `flash` between its narrow, NVS/pkg-preserving erase and
# the full mass-erase + UICR-unlock recovery path.
INIT ?= 0

all: run

nodem-ffi:
	cd nodem-ffi && cargo build --release --target thumbv8m.main-none-eabi --features $(CARGO_LOG_FEATURE)

build: nodem-ffi
	$(NCS_ENV) && west build -b $(BOARD) -d $(ACTIVE_BUILD_DIR) zephyr-app $(EXTRA_CONF_ARGS)

flash:
ifeq ($(INIT),1)
	$(NRFUTIL) device erase --all --family $(FAMILY)
	$(NRFUTIL) device program --firmware $(ACTIVE_MERGED_HEX) --options chip_erase_mode=ERASE_NONE --family $(FAMILY)
	$(NRFUTIL) device write --address $(APPROTECT_ADDR) --value $(APPROTECT_OPEN_VALUE) --family $(FAMILY)
	$(NRFUTIL) device write --address $(SECUREAPPROTECT_ADDR) --value $(APPROTECT_OPEN_VALUE) --family $(FAMILY)
	@echo
	@echo "Flashed (full chip erase, NVS/pkg wiped) - power-cycle the board now (a debugger reset alone won't boot it)."
else
	$(NRFUTIL) device program --firmware $(ACTIVE_MERGED_HEX) --options chip_erase_mode=ERASE_RANGES_TOUCHED_BY_FIRMWARE --family $(FAMILY)
	@echo
	@echo "Flashed (NVS/pkg preserved) - power-cycle the board now (a debugger reset alone won't boot it)."
endif

reset:
	$(NRFUTIL) device reset --reset-kind RESET_SYSTEM --family $(FAMILY)

logs:
ifeq ($(LOG),rtt)
	JLinkRTTLogger -Device $(JLINK_DEVICE) -If SWD -Speed $(RTT_SPEED_KHZ) -RTTChannel 0 $(RTT_LOG_FILE) &
	sleep 2
	tail -f $(RTT_LOG_FILE)
else
	@echo "⏳ Starting logging from $(CONSOLE_TTY)..."
	@stty -F $(CONSOLE_TTY) $(CONSOLE_BAUD) raw -echo
	@cat $(CONSOLE_TTY)
endif

reconfigure:
	@echo "⏳ Sending configuration commands..."
	@./scripts/reconfigure.sh $(RECONFIGURE_CMDS) $(CMD_TTY) $(CMD_BAUD)

run: build flash reset sleep reconfigure logs

sleep:
	@echo "⏳ Sleeping.."
	@sleep 2

clean:
	rm -rf $(BUILD_DIR) $(BUILD_RTT_DIR)
	cd nodem-ffi && cargo clean
