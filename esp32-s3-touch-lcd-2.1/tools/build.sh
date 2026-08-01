#!/usr/bin/env bash
# Build the ESP32-S3-Touch-LCD-2.1 demo firmware.
#
# Usage: tools/build.sh [clean]
#
# Expects ESP-IDF v5.3 or newer. If IDF_PATH is not exported yet, the script
# looks in the usual place (~/esp/esp-idf) and sources export.sh for you.

set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_DIR"

if ! command -v idf.py >/dev/null 2>&1; then
    IDF_PATH="${IDF_PATH:-$HOME/esp/esp-idf}"
    if [ -f "$IDF_PATH/export.sh" ]; then
        echo "Sourcing $IDF_PATH/export.sh"
        # shellcheck disable=SC1091
        . "$IDF_PATH/export.sh"
    else
        echo "ESP-IDF not found. Run tools/install_esp_idf.sh first, or set IDF_PATH." >&2
        exit 1
    fi
fi

if [ "${1:-}" = "clean" ]; then
    idf.py fullclean
    rm -f sdkconfig
fi

idf.py set-target esp32s3
idf.py build

echo
echo "Firmware: $PROJECT_DIR/build/esp32s3_lcd_demo.bin"
echo "Flash it with: tools/flash.sh /dev/ttyACM0"
