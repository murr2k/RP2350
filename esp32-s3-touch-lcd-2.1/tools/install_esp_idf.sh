#!/usr/bin/env bash
# Install ESP-IDF v5.3 for the ESP32-S3 target.
#
# Work-alike of tools/install_pico_sdk.sh in the repository root. Ubuntu 22.04
# or WSL2, same as the RP2350 instructions.

set -euo pipefail

IDF_VERSION="${IDF_VERSION:-v5.3.2}"
IDF_DIR="${IDF_DIR:-$HOME/esp/esp-idf}"

echo "Installing ESP-IDF $IDF_VERSION into $IDF_DIR"

sudo apt update
sudo apt install -y git wget flex bison gperf python3 python3-pip python3-venv \
    cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0

mkdir -p "$(dirname "$IDF_DIR")"

if [ -d "$IDF_DIR/.git" ]; then
    echo "ESP-IDF already cloned, fetching $IDF_VERSION"
    git -C "$IDF_DIR" fetch --tags origin
    git -C "$IDF_DIR" checkout "$IDF_VERSION"
    git -C "$IDF_DIR" submodule update --init --recursive
else
    git clone -b "$IDF_VERSION" --recursive https://github.com/espressif/esp-idf.git "$IDF_DIR"
fi

"$IDF_DIR/install.sh" esp32s3

cat <<EOF

Done. Load the toolchain into your shell with:

    . "$IDF_DIR/export.sh"

Add this to ~/.bashrc if you want it every session:

    alias get_idf='. $IDF_DIR/export.sh'

Then build with:

    cd esp32-s3-touch-lcd-2.1 && idf.py set-target esp32s3 && idf.py build

Serial access on Linux needs your user in the dialout group:

    sudo usermod -aG dialout \$USER

On WSL2, attach the board first from an admin PowerShell:

    usbipd list
    usbipd bind --busid <BUSID>
    usbipd attach --wsl --busid <BUSID>
EOF
