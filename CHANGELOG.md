# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

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

[Unreleased]: https://github.com/murr2k/RP2350/compare/v1.0.0...HEAD
[1.0.0]: https://github.com/murr2k/RP2350/releases/tag/v1.0.0