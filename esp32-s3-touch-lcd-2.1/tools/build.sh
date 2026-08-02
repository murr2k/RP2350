#!/usr/bin/env bash
# Build the ESP32-S3-Touch-LCD-2.1 demo firmware.
#
# Usage: tools/build.sh [clean|<idf.py arguments>]
#
# Takes whichever toolchain the host has. A native ESP-IDF is used if one is on
# PATH or sitting in the usual place; otherwise it falls back to the pinned
# container, which needs nothing installed but Docker and is what the firmware
# on the bench was built with. Either way it has to be v5.3.x, because the port
# uses the i2c_master driver API that v5.2 does not have.
#
# Set BUILD_WITH=docker or BUILD_WITH=native to insist on one of them.

set -euo pipefail

IDF_IMAGE="${IDF_IMAGE:-espressif/idf:v5.3.2}"

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_DIR"

# "clean" is ours; anything else goes through to idf.py untouched.
CLEAN=0
if [ "${1:-}" = "clean" ]; then
    CLEAN=1
    shift
fi
IDF_ARGS=("$@")
if [ ${#IDF_ARGS[@]} -eq 0 ]; then
    IDF_ARGS=(build)
fi

have_native() {
    command -v idf.py >/dev/null 2>&1 && return 0
    [ -f "${IDF_PATH:-$HOME/esp/esp-idf}/export.sh" ]
}

choose() {
    case "${BUILD_WITH:-}" in
        docker|native) echo "$BUILD_WITH"; return ;;
    esac
    if have_native; then echo native; else echo docker; fi
}

build_native() {
    if ! command -v idf.py >/dev/null 2>&1; then
        IDF_PATH="${IDF_PATH:-$HOME/esp/esp-idf}"
        echo "Sourcing $IDF_PATH/export.sh"
        # shellcheck disable=SC1091
        . "$IDF_PATH/export.sh"
    fi
    if [ "$CLEAN" = 1 ]; then
        idf.py fullclean || true
        rm -f sdkconfig
    fi
    idf.py set-target esp32s3
    idf.py "${IDF_ARGS[@]}"
}

build_docker() {
    if ! docker info >/dev/null 2>&1; then
        echo "Docker is installed but not running. Start Docker Desktop." >&2
        exit 1
    fi
    if ! docker image inspect "$IDF_IMAGE" >/dev/null 2>&1; then
        echo "Pulling $IDF_IMAGE (about 8 GB, once)..."
        docker pull "$IDF_IMAGE"
    fi

    # Git Bash rewrites anything that looks like a Unix path before Docker sees
    # it, which turns -w /project into a Windows directory and fails the run.
    # Turning that off and handing Docker a native path avoids it.
    local host_dir="$PROJECT_DIR"
    if [ -n "${MSYSTEM:-}" ]; then
        export MSYS_NO_PATHCONV=1
        host_dir="$(pwd -W)"
    fi

    if [ "$CLEAN" = 1 ]; then
        rm -rf build sdkconfig
    fi

    # Interactive only when there is a terminal, so menuconfig works by hand and
    # this still runs under CI or a pipe.
    local tty_flags=()
    if [ -t 0 ] && [ -t 1 ]; then
        tty_flags=(-it)
    fi

    docker run --rm "${tty_flags[@]}" \
        -v "${host_dir}:/project" -w /project \
        "$IDF_IMAGE" idf.py "${IDF_ARGS[@]}"
}

case "$(choose)" in
    native)
        echo "Building with the host's ESP-IDF."
        build_native
        ;;
    docker)
        if ! command -v docker >/dev/null 2>&1; then
            echo "Neither ESP-IDF nor Docker found. See README.md, or run" >&2
            echo "tools/doctor.sh to see what is missing." >&2
            exit 1
        fi
        echo "No local ESP-IDF, building in $IDF_IMAGE."
        build_docker
        ;;
esac

echo
echo "Firmware: $PROJECT_DIR/build/esp32s3_lcd_demo.bin"
echo "Flash it with: tools/flash.sh [port]"
