#!/usr/bin/env bash
# Flash and monitor the ESP32-S3-Touch-LCD-2.1 demo firmware.
#
# Usage: tools/flash.sh [port]
#
# The RP2350 equivalent was flash_rp2350.sh, which copied a .uf2 onto the
# bootloader drive. The ESP32-S3 has no such drive: it appears as a serial port
# (USB Serial/JTAG) and esptool writes over it.
#
# If the board does not respond, hold BOOT, tap RESET, release BOOT to force
# the ROM bootloader, then run this again.

set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_DIR"

PORT="${1:-}"

if ! command -v idf.py >/dev/null 2>&1; then
    IDF_PATH="${IDF_PATH:-$HOME/esp/esp-idf}"
    if [ -f "$IDF_PATH/export.sh" ]; then
        # shellcheck disable=SC1091
        . "$IDF_PATH/export.sh"
    else
        echo "ESP-IDF not found. Set IDF_PATH or run tools/install_esp_idf.sh." >&2
        exit 1
    fi
fi

if [ -z "$PORT" ]; then
    echo "No port given, letting esptool pick one."
    idf.py flash monitor
else
    idf.py -p "$PORT" flash monitor
fi
