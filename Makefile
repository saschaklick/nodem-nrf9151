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
#   make ota          - build a release OTA image (see its own note below)
#   make flash-ota SLOT=0|1
#                     - flash that image straight into a slot over SWD,
#                       simulating an OTA delivery (see its own note below)
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
#   make ota          - build a signed release image for the MCUboot
#                       secondary (OTA) slot, into its own build-ota/
#                       directory - never the same one `make build`/`flash`
#                       use, so building an OTA image never clobbers (and is
#                       never clobbered by) whatever's currently flashed and
#                       under test on the bench. Sysbuild+MCUboot already
#                       signs every app image unconditionally (see
#                       sysbuild.conf) - this isn't a separate signing step,
#                       just a build whose defaults suit an image meant to
#                       run unattended on a real device rather than one
#                       being watched live over `logs`: LOG_LEVEL defaults
#                       to "error" here (not "debug" - see above), though
#                       it's still the same switch, so `LOG_LEVEL=debug make
#                       ota` works if a debug-logging OTA build is ever
#                       actually wanted. The resulting signed binary is
#                       copied to ota/zephyr.signed.bin (see OTA_OUT_DIR
#                       below) - a stable path outside west's own
#                       build-ota/ tree - and its size is reported against
#                       the 320KB-per-slot budget (pm_static.yml).
#
#   make flash-ota SLOT=0|1 (default 1)
#                     - flashes the last `make ota` build directly into a
#                       slot over SWD (via nrfutil, like plain `flash`) -
#                       standing in for the still-unbuilt real OTA delivery
#                       path (network download + write, see sysbuild.conf's
#                       note) so the swap/rollback machinery can be
#                       exercised now, on the bench, without it. Both slots
#                       flash the exact same underlying image
#                       (build-ota/zephyr-app/zephyr/tfm_merged.hex - the
#                       TF-M+app pair MCUboot swaps as one unit) - only the
#                       destination address, and for SLOT=1 an extra
#                       imgtool `--pad` pass, differ:
#
#                       SLOT=0 - straight into mcuboot_primary_app
#                       (0x8200): overwrites the currently-running image in
#                       place, no swap/test/rollback involved at all -
#                       equivalent to `make flash` but sourced from ota/
#                       instead of build/. Uses zephyr.signed.hex as-is, no
#                       repackaging (it's already addressed there).
#
#                       SLOT=1 - into mcuboot_secondary (0x58000, the real
#                       OTA slot): imgtool signs a *second* copy of the same
#                       image with --pad (not --confirm), so its trailer
#                       lands exactly like a freshly-delivered pending
#                       update would - magic set, image-ok unset (see
#                       bootutil_public.c's boot_swap_tables, the row
#                       matching that exact combination is BOOT_SWAP_TYPE_
#                       TEST) - then that's rebased from a 0-based .bin to a
#                       0x58000-based .hex via objcopy (plain
#                       zephyr.signed.hex's own addresses are baked in for
#                       slot0, not reusable here - .bin has none, so it can
#                       be rebased to either slot). MCUboot performs a
#                       one-time swap into the primary slot on the very
#                       next boot; if the new image never confirms itself
#                       before another reset, MCUboot reverts back to
#                       whatever was in the primary slot before,
#                       automatically. heartbeat_task.c's
#                       ota_confirm_if_due() is the other half of this - it
#                       runs *in* the newly-booted image and calls
#                       boot_write_img_confirmed() once 60s of uptime have
#                       passed, turning that one-time test into a
#                       permanent, confirmed image before anything would
#                       otherwise trigger a revert.
#
#                       Both branches trigger a plain nrfutil reset
#                       afterward rather than requiring a manual
#                       power-cycle - but see the next paragraph's own
#                       caveat for why that's only reliable for SLOT=1, not
#                       SLOT=0.
#
# After flashing, power-cycle the board (unplug/replug USB, or the reset
# button) - a debugger-triggered reset alone does not reliably re-boot this
# nRF9151/TF-M setup into the new image. `flash-ota SLOT=1` is the one
# exception: it never touches the actively-executing primary slot (only the
# inert secondary slot), so this caveat doesn't apply and a plain reset is
# enough to make MCUboot see and swap in the pending image. `flash-ota
# SLOT=0` overwrites the primary slot exactly like plain `flash` does, so
# it's subject to the same caveat - fall back to a manual power-cycle there
# if a reset doesn't bring the new image up.
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

.PHONY: all build nodem-ffi flash reset logs reconfigure run clean ota flash-ota

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

# $(call level_conf,LEVEL) / $(call level_feature,LEVEL) - map a LOG_LEVEL
# name to its zephyr-app/log-*.conf fragment (empty for "debug" - Kconfig's
# own default, no fragment needed) and its nodem-ffi Cargo feature. Factored
# out of the plain ifeq chain this used to be so `make ota` below can map
# its own, separately-defaulted level the same way without duplicating the
# five-way branch.
level_conf = $(if $(filter info,$(1)),log-info.conf,$(if $(filter warn,$(1)),log-warn.conf,$(if $(filter error,$(1)),log-error.conf,$(if $(filter none,$(1)),log-none.conf,))))
level_feature = $(if $(filter info,$(1)),log-info,$(if $(filter warn,$(1)),log-warn,$(if $(filter error,$(1)),log-error,$(if $(filter none,$(1)),log-none,log-debug))))

LOG_LEVEL_CONF    := $(call level_conf,$(LOG_LEVEL))
CARGO_LOG_FEATURE := $(call level_feature,$(LOG_LEVEL))

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

# make ota - see the header comment's own note. Own build directory (never
# BUILD_DIR/BUILD_RTT_DIR) and its own LOG_LEVEL default ("error", not
# "debug" - $(origin) is what lets the *default* differ from `build`'s
# while `LOG_LEVEL=... make ota` on the command line still wins either way,
# same precedence command-line assignments always have over a plain `?=`).
BUILD_OTA_DIR := build-ota

ifeq ($(origin LOG_LEVEL),command line)
OTA_LOG_LEVEL := $(LOG_LEVEL)
else
OTA_LOG_LEVEL := error
endif

OTA_LOG_LEVEL_CONF    := $(call level_conf,$(OTA_LOG_LEVEL))
OTA_CARGO_LOG_FEATURE := $(call level_feature,$(OTA_LOG_LEVEL))
OTA_EXTRA_CONF_ARGS   := $(if $(OTA_LOG_LEVEL_CONF),-- -DEXTRA_CONF_FILE="$(OTA_LOG_LEVEL_CONF)",)

# 320KB - mcuboot_primary/mcuboot_secondary's fixed, by-design size (see
# pm_static.yml's own note on how that number was arrived at); not read
# back from partitions.yml since that file doesn't exist until after the
# very build this checks the output of.
OTA_SLOT_SIZE     := 327680
OTA_SLOT_SIZE_HEX := 0x50000

# mcuboot_primary_app (the running app+TF-M image, inside mcuboot_primary -
# see pm_static.yml) and mcuboot_secondary's own base addresses - both fixed
# by the same static partition layout regardless of build variant (verified
# identical across build/ and build-ota/'s own partitions.yml).
SLOT0_ADDR := 0x8200
SLOT1_ADDR := 0x58000

# SLOT=0|1 for `flash-ota` - see that target's own note.
SLOT ?= 1

ifeq ($(filter $(SLOT),0 1),)
$(error SLOT must be 0 or 1 (got "$(SLOT)"))
endif

# Where `ota` leaves the final signed binary - a plain top-level directory,
# not inside build-ota/ (west's own scratch/CMake tree), so this is the one
# stable path worth pointing a flashing step or a delivery script at,
# regardless of whatever west's own directory layout does across versions.
OTA_OUT_DIR := ota

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

# Target-specific variables below override build's/nodem-ffi's own for this
# whole prerequisite chain (GNU Make propagates a target-specific value down
# to all of that target's prerequisites, recursively) - so `build`'s recipe
# runs unchanged, it just picks up build-ota/ and the error-level log conf
# instead of build/'s own.
ota: LOG_LEVEL_CONF    := $(OTA_LOG_LEVEL_CONF)
ota: CARGO_LOG_FEATURE := $(OTA_CARGO_LOG_FEATURE)
ota: EXTRA_CONF_ARGS   := $(OTA_EXTRA_CONF_ARGS)
ota: ACTIVE_BUILD_DIR  := $(BUILD_OTA_DIR)
ota: build
	@mkdir -p $(OTA_OUT_DIR)
	@cp $(BUILD_OTA_DIR)/zephyr-app/zephyr/zephyr.signed.bin $(OTA_OUT_DIR)/zephyr.signed.bin
	@size=$$(stat -c%s $(OTA_OUT_DIR)/zephyr.signed.bin); \
	pct=$$(( size * 100 / $(OTA_SLOT_SIZE) )); \
	echo; \
	echo "✅ OTA image: $(OTA_OUT_DIR)/zephyr.signed.bin"; \
	echo "   $$size / $(OTA_SLOT_SIZE) bytes ($$pct% of the 320KB OTA slot)"; \
	if [ $$size -gt $(OTA_SLOT_SIZE) ]; then \
		echo "   ⚠️  over the slot size - this will not fit in mcuboot_secondary"; \
	fi

flash-ota:
ifeq ($(SLOT),0)
	$(NRFUTIL) device program --firmware $(BUILD_OTA_DIR)/zephyr-app/zephyr/zephyr.signed.hex --options chip_erase_mode=ERASE_RANGES_TOUCHED_BY_FIRMWARE --family $(FAMILY)
	$(NRFUTIL) device reset --reset-kind RESET_SYSTEM --family $(FAMILY)
	@echo
	@echo "Flashed straight into the primary slot (SLOT=0, $(SLOT0_ADDR)) and reset - no swap involved, so it boots directly, but this overwrote the actively-running image itself: if it doesn't come up, power-cycle (unplug/replug) instead, same as plain \`make flash\` (see its own note - a debugger reset doesn't always see freshly-flashed content in the region it was just executing from)."
else
	$(NCS_ENV) && \
	key=$$(sed -n 's/^CONFIG_BOOT_SIGNATURE_KEY_FILE="\(.*\)"$$/\1/p' $(BUILD_OTA_DIR)/mcuboot/zephyr/.config); \
	python3 "$$ZEPHYR_BASE/../bootloader/mcuboot/scripts/imgtool.py" sign \
		--version 0.0.0+0 --align 4 --pad-header --header-size 0x200 \
		--slot-size $(OTA_SLOT_SIZE_HEX) --pad -k "$$key" \
		$(BUILD_OTA_DIR)/zephyr-app/zephyr/tfm_merged.hex \
		$(BUILD_OTA_DIR)/zephyr-app/zephyr/zephyr.signed.pending.bin && \
	arm-zephyr-eabi-objcopy -I binary -O ihex --change-addresses=$(SLOT1_ADDR) \
		$(BUILD_OTA_DIR)/zephyr-app/zephyr/zephyr.signed.pending.bin \
		$(BUILD_OTA_DIR)/zephyr-app/zephyr/zephyr.signed.pending.hex
	$(NRFUTIL) device program --firmware $(BUILD_OTA_DIR)/zephyr-app/zephyr/zephyr.signed.pending.hex --options chip_erase_mode=ERASE_RANGES_TOUCHED_BY_FIRMWARE --family $(FAMILY)
	$(NRFUTIL) device reset --reset-kind RESET_SYSTEM --family $(FAMILY)
	@echo
	@echo "Flashed a pending (unconfirmed) image into the OTA/secondary slot (SLOT=1, $(SLOT1_ADDR)) and reset to trigger it - unlike SLOT=0/plain \`flash\`, this never touched the actively-executing primary slot, so a plain reset (not a power-cycle) reliably sees it: MCUboot swaps it in as a one-time test on this very reset, and auto-reverts if it's never confirmed (see heartbeat_task.c)."
endif

clean:
	rm -rf $(BUILD_DIR) $(BUILD_RTT_DIR) $(BUILD_OTA_DIR) $(OTA_OUT_DIR)
	cd nodem-ffi && cargo clean
