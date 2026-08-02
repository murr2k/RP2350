#!/usr/bin/env bash
# Flash the ESP32-S3-Touch-LCD-2.1 demo firmware.
#
# Usage: tools/flash.sh [port]
#        tools/flash.sh              # lists ports if it cannot pick one
#        tools/flash.sh COM3
#        tools/flash.sh /dev/ttyACM0
#
# This always runs on the host, never in the container, even when the build was
# containerised. Docker Desktop on Windows and macOS cannot pass a serial port
# through to a container, so a build image has no way to reach the board.
#
# The RP2350 equivalent copied a .uf2 onto a bootloader drive. The ESP32-S3 has
# no such drive: it appears as a serial port and esptool writes over it, using
# the auto reset lines so the board does not need putting into download mode by
# hand. If it does not answer, hold BOOT, tap RESET, release BOOT, and retry.

set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_DIR"

PORT="${1:-}"
BAUD="${BAUD:-460800}"

# Windows ships a "python3" that exists only to advertise the Microsoft Store:
# it resolves on PATH and then refuses to run, so each candidate is executed
# rather than merely looked up.
PY=""
for candidate in python3 python py; do
    if command -v "$candidate" >/dev/null 2>&1 &&
       "$candidate" -c "import sys" >/dev/null 2>&1; then
        PY="$candidate"
        break
    fi
done
if [ -z "$PY" ]; then
    echo "No working python on PATH. See README.md, or run tools/doctor.sh." >&2
    exit 1
fi

if ! "$PY" -m esptool version >/dev/null 2>&1; then
    echo "esptool is not installed. Install it with:" >&2
    echo "    $PY -m pip install esptool pyserial" >&2
    exit 1
fi

if [ ! -f build/flash_args ]; then
    echo "Nothing built yet. Run tools/build.sh first." >&2
    exit 1
fi

if [ -z "$PORT" ]; then
    echo "No port given. Ports currently present:"
    "$PY" -c "
import serial.tools.list_ports as lp
ports = list(lp.comports())
for p in ports:
    print('   ', p.device, '-', p.description)
if not ports:
    print('    (none: check the cable, and use the socket marked UART)')
" || true
    echo
    echo "Usage: tools/flash.sh <port>"
    exit 1
fi

# esptool renamed its subcommands to hyphens at version 5 and the ESP-IDF build
# still prints the old spelling in its instructions. Pick whichever this one
# understands rather than depending on the host having a particular version.
MAJOR="$("$PY" -m esptool version 2>/dev/null |
         grep -oE '[0-9]+\.[0-9]+\.[0-9]+' | head -1 | cut -d. -f1)"
if [ "${MAJOR:-5}" -ge 5 ]; then
    WRITE=write-flash
else
    WRITE=write_flash
fi

cd build
exec "$PY" -m esptool --chip esp32s3 -p "$PORT" -b "$BAUD" \
    --before default_reset --after hard_reset "$WRITE" "@flash_args"
