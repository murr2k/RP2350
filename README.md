# RP2350 LCD & IMU Demo Collection

A comprehensive collection of demos and utilities for the Waveshare RP2350-LCD-1.28 development board, featuring a round 240x240 LCD display and QMI8658 6-axis IMU.

## Author
Murray Kopit (murr2k@gmail.com)  
Date: August 24, 2025  
License: MIT

## Ports

| Board | Directory | Toolchain |
|-------|-----------|-----------|
| Waveshare RP2350-LCD-1.28 (240x240 round) | this directory | Pico SDK 2.0.0 |
| Waveshare ESP32-S3-Touch-LCD-2.1 (480x480 round, touch) | [`esp32-s3-touch-lcd-2.1/`](esp32-s3-touch-lcd-2.1/) | ESP-IDF 5.3 |

The ESP32-S3 port is a work-alike: the same demos with the same serial command
grammars, so the Python test tools in this directory drive either board. See
[esp32-s3-touch-lcd-2.1/PORTING_NOTES.md](esp32-s3-touch-lcd-2.1/PORTING_NOTES.md)
for the hardware and API mapping.

## Hardware

This project is designed for the [Waveshare RP2350-LCD-1.28](https://www.waveshare.com/wiki/RP2350-LCD-1.28) development board featuring:
- **MCU**: Raspberry Pi RP2350 (Dual ARM Cortex-M33)
- **Display**: 1.28" Round LCD (240×240, GC9A01A controller, SPI interface)
- **IMU**: QMI8658 6-axis sensor (3-axis accelerometer + 3-axis gyroscope, I2C interface)
- **Memory**: 520KB SRAM, 4MB Flash
- **Interfaces**: USB Type-C, GPIO headers, JST battery connector

## Features

### Demos Included

1. **configurable_cube** - Interactive 3D cube with runtime-adjustable IMU mapping via serial CLI
2. **madgwick_cube** - 6-DOF orientation tracking using Madgwick filter
3. **intuitive_cube** - Tilt-responsive cube using accelerometer only
4. **buffered_cube** - Basic rotating wireframe cube demonstration
5. **rotation_test** - IMU axis calibration and testing tool
6. **axis_test** - Visual IMU data display
7. **gravity_locked_cube** - Cube that maintains orientation relative to gravity
8. **Various test utilities** - SPI test, minimal LCD test, etc.

### Key Technologies

- **Madgwick Filter**: Quaternion-based sensor fusion for drift-free orientation
- **Frame Buffer Rendering**: Smooth graphics via buffered display updates
- **Real-time Serial CLI**: Interactive parameter adjustment via USB CDC
- **Hardware Abstraction**: Clean separation of drivers and application code

## Development Environment Setup

### Prerequisites

#### 1. Operating System
- **Ubuntu 20.04/22.04** (native or WSL2)
- For WSL2: Ensure WSLg is enabled for USB passthrough (optional)

#### 2. Install Build Tools
```bash
# Update package list
sudo apt update

# Install essential build tools
sudo apt install -y \
    cmake \
    gcc-arm-none-eabi \
    libnewlib-arm-none-eabi \
    libstdc++-arm-none-eabi-newlib \
    build-essential \
    git \
    python3 \
    python3-pip

# Install additional tools
sudo apt install -y \
    minicom \
    screen
```

#### 3. Install Raspberry Pi Pico SDK
```bash
# Create workspace
mkdir -p ~/pico
cd ~/pico

# Clone Pico SDK
git clone -b 2.0.0 https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk
git submodule update --init

# Set environment variable (add to ~/.bashrc for persistence)
export PICO_SDK_PATH=~/pico/pico-sdk
echo 'export PICO_SDK_PATH=~/pico/pico-sdk' >> ~/.bashrc
```

#### 4. Install Picotool (Optional but recommended)
```bash
cd ~/pico
git clone https://github.com/raspberrypi/picotool.git
cd picotool
mkdir build
cd build
cmake ..
make
sudo make install
```

### Building the Project

```bash
# Clone this repository
git clone https://github.com/murr2k/RP2350.git
cd RP2350

# Create build directory
mkdir build
cd build

# Configure with CMake
cmake ..

# Build all targets
make -j$(nproc)

# Or build specific target
make configurable_cube
```

### Build Output

Successfully built firmwares will be in the `build` directory as `.uf2` files:
- `configurable_cube.uf2` - Main interactive demo
- `madgwick_cube.uf2` - 6-DOF tracking demo
- `rotation_test.uf2` - Axis calibration tool
- And more...

## Flashing Firmware

### Method 1: UF2 Bootloader (Easiest)
1. Hold the BOOT button on the board
2. Connect USB cable (or press RESET while holding BOOT)
3. Release BOOT button
4. Board appears as USB mass storage device (RPI-RP2)
5. Copy desired `.uf2` file to the drive
6. Board automatically reboots with new firmware

### Method 2: Using Picotool
```bash
# List connected RP2350 devices
picotool info

# Flash firmware
picotool load configurable_cube.uf2
picotool reboot
```

## Using the Configurable Cube Demo

### Serial Connection

#### Windows
1. Use PuTTY, TeraTerm, or Arduino Serial Monitor
2. Find COM port in Device Manager
3. Connect at 115200 baud, 8N1

#### Linux/WSL
```bash
# Find device (usually /dev/ttyACM0 or /dev/ttyUSB0)
ls /dev/tty*

# Connect with minicom
minicom -D /dev/ttyACM0 -b 115200

# Or use screen
screen /dev/ttyACM0 115200
```

### CLI Commands

Press 'm' in the terminal to see the menu:

**Axis Controls:**
- `1-6`: Toggle axis inversions (1=AX 2=AY 3=AZ 4=GX 5=GY 6=GZ)
- `q/w`: Swap accelerometer axes
- `a/s`: Swap accelerometer axes
- `z/x`: Swap accelerometer axes

**Display Controls:**
- `d`: Toggle sensor data display
- `c`: Toggle cube display
- `e`: Toggle axes display
- `p`: Pause/unpause

**Filter Controls:**
- `+/-`: Increase/decrease Madgwick beta (convergence rate)
- `r`: Reset orientation to identity quaternion
- `b`: Recalibrate gyroscope bias

**Information:**
- `v`: View current sensor values
- `m`: Show menu
- `i`: Show current configuration

## Project Structure

```
RP2350/
├── CMakeLists.txt          # Main CMake configuration
├── README.md               # This file
├── LICENSE                 # MIT License
├── build/                  # Build output directory (generated)
│   └── *.uf2              # Firmware files
├── src/                    # Source code
│   ├── configurable_cube.c # Main interactive demo
│   ├── madgwick_cube.c     # Madgwick filter implementation
│   ├── rotation_test.c     # Axis testing utility
│   └── ...                 # Other demos
├── lib/                    # Waveshare libraries
│   ├── Config/             # Board configuration
│   └── LCD/                # Display driver
├── include/                # Header files
└── drivers/                # Additional drivers (unused)
```

## Technical Details

### Display Communication
- **Interface**: SPI (10MHz)
- **Chip Select**: Must be held LOW continuously after reset
- **Frame Buffer**: 240×240×2 bytes (115.2KB)
- **Color Format**: RGB565 (16-bit)

### IMU Configuration
- **I2C Address**: 0x6B
- **Accelerometer**: ±2g range, 16384 LSB/g
- **Gyroscope**: ±256 dps range, 128 LSB/dps
- **Sample Rate**: 100-250 Hz

### Madgwick Filter Parameters
- **Beta**: 0.1 (default) - Higher values trust accelerometer more
- **Sample Rate**: 100 Hz
- **Quaternion Output**: Unit quaternion representing orientation

## Troubleshooting

### Black Screen
- Ensure CS pin is held LOW after LCD reset
- Check SPI connections and clock speed
- Verify backlight PWM is set

### IMU Not Responding
- Check I2C pull-up resistors (internal pull-ups enabled)
- Verify QMI8658 WHO_AM_I returns 0x05
- Try I2C address 0x6A if 0x6B fails

### Incorrect Axis Mapping
1. Flash `rotation_test.uf2`
2. Observe which axes respond to rotation
3. Use `configurable_cube.uf2` to find correct mapping
4. Note configuration and update source code

### USB Serial Not Working
- Ensure `pico_enable_stdio_usb(target 1)` in CMakeLists.txt
- Wait 2-3 seconds after reset for USB enumeration
- Check device manager/dmesg for CDC device

## Dependencies

- [Raspberry Pi Pico SDK 2.0.0](https://github.com/raspberrypi/pico-sdk)
- [Waveshare LCD Libraries](https://github.com/waveshare/Pico_code/tree/main/c/lib)
- ARM GCC Toolchain 13.2 or later
- CMake 3.13 or later

## References

- [Waveshare RP2350-LCD-1.28 Wiki](https://www.waveshare.com/wiki/RP2350-LCD-1.28)
- [Waveshare GitHub Repository](https://github.com/waveshare/Pico_code)
- [RP2350 Datasheet](https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf)
- [QMI8658 Datasheet](https://www.waveshare.com/w/upload/1/18/Qmi8658a.pdf)
- [Madgwick Filter Paper](https://www.x-io.co.uk/res/doc/madgwick_internal_report.pdf)

## Contributing

Issues and pull requests are welcome! Please ensure any new demos:
1. Follow the existing code structure
2. Include appropriate error handling
3. Document any new CLI commands
4. Test on actual hardware

## License

MIT License - See [LICENSE](LICENSE) file for details

## Acknowledgments

- Sebastian Madgwick for the IMU filter algorithm
- Waveshare for hardware documentation and base drivers
- Raspberry Pi Foundation for the Pico SDK

---
*Created by Murray Kopit, August 24, 2025*
