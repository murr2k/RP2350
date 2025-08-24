#!/bin/bash

# Install Pico SDK for RP2350 development
# This script sets up the Raspberry Pi Pico SDK with RP2350 support

set -e

echo "=== Installing Raspberry Pi Pico SDK ==="
echo "This will install the SDK and required tools for RP2350 development"

# Set installation directory
PICO_DIR="$HOME/pico"
PICO_SDK_PATH="$PICO_DIR/pico-sdk"
PICO_EXAMPLES_PATH="$PICO_DIR/pico-examples"
PICO_EXTRAS_PATH="$PICO_DIR/pico-extras"
PICO_PLAYGROUND_PATH="$PICO_DIR/pico-playground"

# Install required packages
echo "Installing required packages..."
sudo apt update || true
sudo apt install -y cmake gcc-arm-none-eabi libnewlib-arm-none-eabi \
    libstdc++-arm-none-eabi-newlib build-essential git python3 \
    pkg-config libusb-1.0-0-dev || {
    echo "⚠ Package installation failed - some packages may already be installed"
}

# Create pico directory
echo "Creating Pico directory at $PICO_DIR..."
mkdir -p "$PICO_DIR"
cd "$PICO_DIR"

# Clone Pico SDK if not exists
if [ ! -d "$PICO_SDK_PATH" ]; then
    echo "Cloning Pico SDK..."
    git clone https://github.com/raspberrypi/pico-sdk.git
    cd "$PICO_SDK_PATH"
    git submodule update --init
else
    echo "Pico SDK already exists, updating..."
    cd "$PICO_SDK_PATH"
    git pull
    git submodule update --init --recursive
fi

# Clone Pico Examples if not exists
if [ ! -d "$PICO_EXAMPLES_PATH" ]; then
    echo "Cloning Pico Examples..."
    cd "$PICO_DIR"
    git clone https://github.com/raspberrypi/pico-examples.git
else
    echo "Pico Examples already exists, updating..."
    cd "$PICO_EXAMPLES_PATH"
    git pull
fi

# Clone Pico Extras (optional but useful)
if [ ! -d "$PICO_EXTRAS_PATH" ]; then
    echo "Cloning Pico Extras..."
    cd "$PICO_DIR"
    git clone https://github.com/raspberrypi/pico-extras.git
else
    echo "Pico Extras already exists, updating..."
    cd "$PICO_EXTRAS_PATH"
    git pull
fi

# Clone Pico Playground (optional examples)
if [ ! -d "$PICO_PLAYGROUND_PATH" ]; then
    echo "Cloning Pico Playground..."
    cd "$PICO_DIR"
    git clone https://github.com/raspberrypi/pico-playground.git
else
    echo "Pico Playground already exists, updating..."
    cd "$PICO_PLAYGROUND_PATH"
    git pull
fi

# Set up environment variables
echo "Setting up environment variables..."
echo "" >> ~/.bashrc
echo "# Raspberry Pi Pico SDK" >> ~/.bashrc
echo "export PICO_SDK_PATH=$PICO_SDK_PATH" >> ~/.bashrc
echo "export PICO_EXAMPLES_PATH=$PICO_EXAMPLES_PATH" >> ~/.bashrc
echo "export PICO_EXTRAS_PATH=$PICO_EXTRAS_PATH" >> ~/.bashrc
echo "export PICO_PLAYGROUND_PATH=$PICO_PLAYGROUND_PATH" >> ~/.bashrc

# Create a simple env file for the project
ENV_FILE="/home/murr2k/projects/RP2350/pico_sdk.env"
cat > "$ENV_FILE" << EOF
# Pico SDK Environment Variables
export PICO_SDK_PATH=$PICO_SDK_PATH
export PICO_EXAMPLES_PATH=$PICO_EXAMPLES_PATH
export PICO_EXTRAS_PATH=$PICO_EXTRAS_PATH
export PICO_PLAYGROUND_PATH=$PICO_PLAYGROUND_PATH
EOF

echo "✓ Pico SDK installed at: $PICO_SDK_PATH"
echo "✓ Pico Examples at: $PICO_EXAMPLES_PATH"
echo "✓ Pico Extras at: $PICO_EXTRAS_PATH"
echo "✓ Pico Playground at: $PICO_PLAYGROUND_PATH"
echo ""
echo "Environment variables have been added to ~/.bashrc"
echo "To use them in current session, run: source ~/.bashrc"
echo "Or for this project only: source /home/murr2k/projects/RP2350/pico_sdk.env"
echo ""
echo "=== Pico SDK Installation Complete ==="