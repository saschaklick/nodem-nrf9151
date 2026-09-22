#!/usr/bin/env bash
#
# Sends each line of a commands file to the firmware's process_command UART
# (uart0 - see zephyr-app/src/uart_task.c - NOT the console UART `make logs`
# reads) as a "#..." command, printing whatever the device replies to each
# so failures are visible immediately rather than only inferred later.
#
# Meant to run right after `make flash` + a power cycle: every flash mass-
# erases the whole chip (see the Makefile's own comment on `flash`), which
# wipes config_store's persisted state (device_id/device_secret/reg_code/
# device_name/...) along with everything else. The commands file is yours
# to write - whatever sequence actually gets the device back to the state
# you want, e.g. re-registering with a fresh code, or (once a command for
# it exists) restoring a known device_id/secret directly without spending
# another registration code on it. There's deliberately no fixed command
# set baked in here.
#
# Usage: scripts/reconfigure.sh <commands-file> [tty] [baud]

set -euo pipefail

CMDS_FILE="${1:?usage: $0 <commands-file> [tty] [baud]}"
TTY="${2:-/dev/ttyACM0}"
BAUD="${3:-115200}"

if [ ! -f "$CMDS_FILE" ]; then
	echo "No such commands file: $CMDS_FILE" >&2
	exit 1
fi

stty -F "$TTY" "$BAUD" raw -echo

# Runs for as long as this script does, printing replies as they arrive
# rather than only after every command has been sent - each command's
# result (line_bytes) is a single "<line>\r\n" per the protocol, read here
# one reply at a time (not a raw `cat` passthrough) so each can be indented
# 3 spaces - the width of a sent line's " → " prefix - so a reply's text
# lines up directly under the command that produced it, with no symbol of
# its own (only sent lines are marked, so an unmarked line is always a
# reply). `${reply%$'\r'}` strips the trailing \r the protocol always sends,
# so it doesn't fight the terminal's own \n on the next line.
while IFS= read -r reply || [ -n "$reply" ]; do
	printf '   %s\n' "${reply%$'\r'}"
done < "$TTY" &
READER_PID=$!
trap 'kill "$READER_PID" 2>/dev/null || true' EXIT

while IFS= read -r line || [ -n "$line" ]; do
	# Blank lines, and "//" comments - not "#", which would collide with
	# commands themselves all starting with that.
	[ -z "$line" ] && continue
	case "$line" in
	//*) continue ;;
	esac

	printf ' → %s\n' "$line"
	printf '%s\r\n' "$line" > "$TTY"
	sleep 0.2
done < "$CMDS_FILE"

# Gives the last command's reply time to arrive before the trap above kills
# the background reader.
sleep 0.5
