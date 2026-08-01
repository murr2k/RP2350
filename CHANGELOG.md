# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- **ESP32-S3-Touch-LCD-2.1 port** (`esp32-s3-touch-lcd-2.1/`): a work-alike of
  the whole demo collection for the Waveshare 2.1" round 480x480 touch board,
  built with ESP-IDF instead of the Pico SDK. Same demos, same serial command
  grammars, so the existing Python test tools drive it unchanged.
  - ST7701S parallel RGB display driver with double buffering in PSRAM
  - CST820 touch driver and an on-device demo launcher, replacing "flash a
    different .uf2" with runtime demo selection
  - 5x7 font and drawing helpers, so demos show numbers as well as bar graphs
  - `PORTING_NOTES.md` documenting the hardware and API mapping
- **QMI8658 range fix** carried by the port: the RP2350 sources configure
  +/-512 dps while dividing by the +/-256 dps scale factor, halving every gyro
  reading. The ESP32-S3 driver configures the range the demos assume.

## [2.0.0] - 2025-08-26

### Added
- **Advanced Kalman Filter Implementations:**
  - `kalman_6dof_configurable` - Full 6-DOF quaternion-based Kalman filter with runtime axis configuration
  - `kalman_6dof` - Optimized quaternion Kalman filter with drift-free orientation tracking
  - `kalman_fixed` - Fixed-point optimized Kalman implementation
  - `kalman_balance` - Specialized filter for balancing applications
  - `quaternion_kalman` - Extended Kalman Filter (EKF) with quaternion state representation
- **Regression Testing Framework:**
  - IMU data injection system for closed-loop debugging
  - 8-quadrant motion pattern testing
  - Automated sensor fusion validation
  - Real-time plotting and visualization tools
  - CSV data export for analysis
- **Serial Bridge System:**
  - TCP-to-Serial bridge for reliable WSL communication
  - Windows-side Python bridge with graceful Ctrl-C shutdown
  - Comprehensive connection documentation
  - Multiple connection methods support
- **Enhanced Command System:**
  - `I<ax>,<ay>,<az>,<gx>,<gy>,<gz>` - Direct IMU data injection
  - `AMAP=x,y,z` - Runtime axis remapping
  - `AG[0-2]=±1` - Gyro axis sign configuration
  - `AS[0-2]=scale` - Gyro axis scaling
  - `T[XYZ]=amplitude` - Test signal generation
  - Extended help system with command examples
- **Development Tools:**
  - `flash_rp2350.sh` - Automated flashing script with picotool
  - `check_usb.sh` - USB device status checker
  - Multiple Python test utilities for validation
  - Comprehensive theory of operation documentation

### Fixed
- Critical Kalman filter bugs:
  - Bias reset issue that was zeroing calibrated values
  - Incorrect gyro scaling (fixed to 128 LSB/dps)
  - Missing gyro range configuration in QMI8658 initialization
  - Axis mapping errors causing pitch/roll swap
  - Display initialization for 3D cube rendering
- USB passthrough issues with usbipd attachment procedures
- Serial communication hanging in WSL

### Changed
- Improved CMake configuration for better build reliability
- Enhanced .gitignore with Python and test file patterns
- Refactored command processing for better extensibility
- Optimized quaternion integration algorithms

### Documentation
- `THEORY_OF_OPERATION.md` - Complete system architecture and algorithms
- `SERIAL_INTERFACE_DETAILED.md` - USB CDC implementation details
- `SERIAL_BRIDGE_METHOD.md` - TCP bridge setup and usage guide
- Comprehensive inline code documentation

## [1.0.0] - 2025-08-24

### Added
- Initial release of RP2350 LCD & IMU Demo Collection
- **Core Demos:**
  - `configurable_cube` - Interactive 3D cube with runtime-adjustable IMU mapping via serial CLI
  - `madgwick_cube` - 6-DOF orientation tracking using Madgwick filter algorithm
  - `intuitive_cube` - Tilt-responsive cube using accelerometer only
  - `buffered_cube` - Basic rotating wireframe cube with frame buffer rendering
  - `gravity_locked_cube` - Cube that maintains orientation relative to gravity
- **Calibration and Test Tools:**
  - `rotation_test` - IMU axis calibration tool with visual feedback
  - `axis_test` - Real-time IMU data visualization
  - `spi_test` - SPI communication testing utility
  - `minimal_lcd` - Basic LCD functionality test
  - `waveshare_test` - Waveshare library validation
- **IMU Features:**
  - QMI8658 6-axis IMU driver implementation (I2C interface)
  - Quaternion-based sensor fusion with Madgwick filter
  - Runtime configurable axis mapping and inversion
  - Gyroscope bias calibration
  - Complementary filter implementation for gravity tracking
- **Display Features:**
  - GC9A01A round LCD driver (240×240 pixels, SPI interface)
  - Frame buffer rendering for smooth animation
  - Optimized SPI communication with continuous CS assertion
  - RGB565 color format support
  - Wireframe 3D graphics rendering with Bresenham line algorithm
- **Interactive CLI System:**
  - Real-time parameter adjustment via USB CDC serial
  - Axis inversion toggles (6 axes)
  - Display element toggles (cube, axes, sensor data)
  - Madgwick beta gain adjustment
  - Orientation reset and gyro recalibration commands
  - Live sensor value monitoring
- **Development Infrastructure:**
  - CMake build system for Raspberry Pi Pico SDK 2.0.0
  - Comprehensive build instructions for Ubuntu/WSL2
  - Detailed hardware setup documentation
  - MIT License
  - Professional project structure with separated drivers and libraries

### Technical Specifications
- **MCU:** Raspberry Pi RP2350 (Dual ARM Cortex-M33 @ 150MHz)
- **Display:** 1.28" Round LCD, 240×240, 65K colors, GC9A01A controller
- **IMU:** QMI8658 6-axis (±2g accelerometer, ±256dps gyroscope)
- **Interfaces:** USB Type-C (CDC serial), SPI (display), I2C (IMU)
- **Memory:** 520KB SRAM, 4MB Flash
- **Update Rate:** 100Hz sensor fusion, 20-30 FPS graphics

### Known Issues
- USB serial port may not appear in WSL2 without proper USB passthrough configuration
- Some IMU axis mappings may need adjustment depending on board mounting orientation

### Dependencies
- Raspberry Pi Pico SDK 2.0.0
- ARM GCC Toolchain 13.2+
- CMake 3.13+
- Waveshare LCD libraries (included)

[Unreleased]: https://github.com/murr2k/RP2350/compare/v2.0.0...HEAD
[2.0.0]: https://github.com/murr2k/RP2350/compare/v1.0.0...v2.0.0
[1.0.0]: https://github.com/murr2k/RP2350/releases/tag/v1.0.0