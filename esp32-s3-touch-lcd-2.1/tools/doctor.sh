#!/usr/bin/env bash
# Check a host can build, flash and talk to the board.
#
# Usage: tools/doctor.sh
#
# Reports what is present and what is missing, and says what to do about each.
# Nothing here changes anything.

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_DIR"

PASS=0
WARN=0
FAIL=0

ok()   { printf '  ok    %s\n' "$1"; PASS=$((PASS + 1)); }
warn() { printf '  note  %s\n' "$1"; WARN=$((WARN + 1)); }
bad()  { printf '  MISS  %s\n' "$1"; FAIL=$((FAIL + 1)); }

# Windows ships a "python3" that exists only to advertise the Microsoft Store:
# it resolves on PATH and then refuses to run. So each candidate is actually
# executed rather than merely looked up.
pick_python() {
    for candidate in python3 python py; do
        if command -v "$candidate" >/dev/null 2>&1 &&
           "$candidate" -c "import sys" >/dev/null 2>&1; then
            echo "$candidate"
            return
        fi
    done
}
PY="$(pick_python)"

echo
echo "Toolchain, one of these two is enough"
echo "-------------------------------------"

NATIVE=0
DOCKER=0

if command -v idf.py >/dev/null 2>&1; then
    ok "ESP-IDF on PATH: $(idf.py --version 2>/dev/null | head -1)"
    NATIVE=1
elif [ -f "${IDF_PATH:-$HOME/esp/esp-idf}/export.sh" ]; then
    ok "ESP-IDF present at ${IDF_PATH:-$HOME/esp/esp-idf}, not yet sourced"
    NATIVE=1
else
    warn "no native ESP-IDF (tools/install_esp_idf.sh installs one on Linux)"
fi

if command -v docker >/dev/null 2>&1; then
    if docker info >/dev/null 2>&1; then
        ok "Docker running: $(docker --version)"
        if docker image inspect espressif/idf:v5.3.2 >/dev/null 2>&1; then
            ok "espressif/idf:v5.3.2 image present"
        else
            warn "image not pulled yet: docker pull espressif/idf:v5.3.2 (8 GB)"
        fi
        DOCKER=1
    else
        warn "Docker installed but not running: start Docker Desktop"
    fi
else
    warn "no Docker (https://docs.docker.com/get-docker/)"
fi

if [ "$NATIVE" = 0 ] && [ "$DOCKER" = 0 ]; then
    bad "nothing can build the firmware: install Docker, or ESP-IDF v5.3.x"
fi

echo
echo "Flashing and console, all run on the host"
echo "-----------------------------------------"

if [ -n "$PY" ]; then
    ok "python: $("$PY" --version 2>&1) (as \"$PY\")"
    if "$PY" -m esptool version >/dev/null 2>&1; then
        ok "esptool: $("$PY" -m esptool version 2>/dev/null | grep -oE '[0-9]+\.[0-9]+\.[0-9]+' | head -1)"
    else
        bad "esptool: $PY -m pip install esptool"
    fi
    if "$PY" -c "import serial" >/dev/null 2>&1; then
        ok "pyserial: $("$PY" -c 'import serial; print(serial.__version__)')"
    else
        bad "pyserial: $PY -m pip install pyserial"
    fi
else
    bad "no python: needed for esptool, which is the only way to flash this chip"
fi

echo
echo "Board"
echo "-----"

if [ -n "$PY" ] && "$PY" -c "import serial" >/dev/null 2>&1; then
    PORTS="$("$PY" -c "
import serial.tools.list_ports as lp
for p in lp.comports():
    print('%s|%s' % (p.device, p.description))
" 2>/dev/null)"
    if [ -n "$PORTS" ]; then
        while IFS='|' read -r dev desc; do
            ok "serial port $dev  ($desc)"
        done <<< "$PORTS"
        echo "        use the socket marked UART, not the one marked USB"
    else
        warn "no serial ports. Board unplugged, a charge-only cable, or on"
        echo "        Windows the CH343 driver is missing (wch.cn), and on Linux"
        echo "        your user may not be in the dialout group"
    fi
else
    warn "cannot list serial ports without pyserial"
fi

echo
echo "Project"
echo "-------"

if [ -f main/wifi_secrets.h ]; then
    ok "main/wifi_secrets.h present, the clock will try to sync"
else
    warn "no main/wifi_secrets.h: builds and runs, but the clock stays unset."
    echo "        cp main/wifi_secrets.h.example main/wifi_secrets.h and edit it"
fi

if [ -f build/flash_args ]; then
    ok "build/ present, ready to flash"
else
    warn "nothing built yet: tools/build.sh"
fi

echo
printf 'ok %d, notes %d, missing %d\n' "$PASS" "$WARN" "$FAIL"
echo
if [ "$FAIL" -gt 0 ]; then
    echo "Something needed is missing, see MISS above."
    exit 1
fi
echo "Good to build."
