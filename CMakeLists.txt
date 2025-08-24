cmake_minimum_required(VERSION 3.13)

# Set board type
set(PICO_BOARD pico2)

# Initialize the Pico SDK
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(rp2350_lcd_demo C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the Pico SDK
pico_sdk_init()

# Project version
set(PROJECT_VERSION_MAJOR 1)
set(PROJECT_VERSION_MINOR 0)
set(PROJECT_VERSION_PATCH 0)

# Compiler options
add_compile_options(-Wall -Wextra -Werror)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/lib
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/display
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/sensors
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/common
)

# Add subdirectories
add_subdirectory(src)
add_subdirectory(drivers)
add_subdirectory(lib)

# Main executable
add_executable(rp2350_lcd_demo
    src/rp2350_demo.c
    drivers/sensors/qmi8658.c
    drivers/display/gc9a01.c
)

# Diagnostic executable
add_executable(rp2350_diagnostic
    src/rp2350_diagnostic.c
)

# Link libraries for diagnostic
target_link_libraries(rp2350_diagnostic
    pico_stdlib
    hardware_spi
    hardware_gpio
    hardware_pwm
)

# Enable USB output for diagnostic
pico_enable_stdio_usb(rp2350_diagnostic 1)
pico_enable_stdio_uart(rp2350_diagnostic 0)

# Create map/bin/hex/uf2 files for diagnostic
pico_add_extra_outputs(rp2350_diagnostic)

# Link libraries
target_link_libraries(rp2350_lcd_demo
    pico_stdlib
    pico_multicore
    hardware_spi
    hardware_i2c
    hardware_adc
    hardware_gpio
    hardware_pwm
    hardware_timer
    pico_unique_id
    display_driver
    sensor_drivers
    system_utils
)

# Enable USB output, disable UART output
pico_enable_stdio_usb(rp2350_lcd_demo 1)
pico_enable_stdio_uart(rp2350_lcd_demo 0)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(rp2350_lcd_demo)

# Waveshare demo executable
add_executable(waveshare_demo
    src/waveshare_demo.c
)

# Link libraries for Waveshare demo
target_link_libraries(waveshare_demo
    pico_stdlib
    hardware_spi
    hardware_i2c
    hardware_gpio
    hardware_pwm
    hardware_adc
)

# Enable USB output for Waveshare demo
pico_enable_stdio_usb(waveshare_demo 1)
pico_enable_stdio_uart(waveshare_demo 0)

# Create map/bin/hex/uf2 files for Waveshare demo
pico_add_extra_outputs(waveshare_demo)

# SPI test executable
add_executable(spi_test
    src/spi_test.c
)

target_link_libraries(spi_test
    pico_stdlib
    hardware_spi
    hardware_gpio
    hardware_pwm
)

pico_enable_stdio_usb(spi_test 1)
pico_enable_stdio_uart(spi_test 0)
pico_add_extra_outputs(spi_test)

# Waveshare fixed executable
add_executable(waveshare_fixed
    src/waveshare_fixed.c
)

target_link_libraries(waveshare_fixed
    pico_stdlib
    hardware_spi
    hardware_gpio
    hardware_pwm
)

pico_enable_stdio_usb(waveshare_fixed 1)
pico_enable_stdio_uart(waveshare_fixed 0)
pico_add_extra_outputs(waveshare_fixed)

# IMU Display Demo executable
add_executable(imu_display_demo
    src/imu_display_demo.c
)

target_link_libraries(imu_display_demo
    pico_stdlib
    hardware_spi
    hardware_i2c
    hardware_gpio
    hardware_pwm
    hardware_adc
)

pico_enable_stdio_usb(imu_display_demo 1)
pico_enable_stdio_uart(imu_display_demo 0)
pico_add_extra_outputs(imu_display_demo)

# Gravity-locked cube demo
add_executable(gravity_cube
    src/gravity_cube.c
)

target_link_libraries(gravity_cube
    pico_stdlib
    hardware_spi
    hardware_i2c
    hardware_gpio
    hardware_pwm
)

pico_enable_stdio_usb(gravity_cube 1)
pico_enable_stdio_uart(gravity_cube 0)
pico_add_extra_outputs(gravity_cube)

# Wireframe cube demo
add_executable(wireframe_cube
    src/wireframe_cube.c
)

target_link_libraries(wireframe_cube
    pico_stdlib
    hardware_spi
    hardware_i2c
    hardware_gpio
    hardware_pwm
)

pico_enable_stdio_usb(wireframe_cube 1)
pico_enable_stdio_uart(wireframe_cube 0)
pico_add_extra_outputs(wireframe_cube)

# Simple cube test (no IMU)
add_executable(simple_cube
    src/simple_cube.c
)

target_link_libraries(simple_cube
    pico_stdlib
    hardware_spi
    hardware_gpio
    hardware_pwm
)

pico_enable_stdio_usb(simple_cube 1)
pico_enable_stdio_uart(simple_cube 0)
pico_add_extra_outputs(simple_cube)

# Waveshare cube demo using official driver
add_executable(waveshare_cube
    src/waveshare_cube.c
    lib/LCD/LCD_1in28.c
    lib/Config/DEV_Config.c
)

target_include_directories(waveshare_cube PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/LCD
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/Config
)

target_link_libraries(waveshare_cube
    pico_stdlib
    hardware_spi
    hardware_i2c
    hardware_gpio
    hardware_pwm
    hardware_adc
)

pico_enable_stdio_usb(waveshare_cube 1)
pico_enable_stdio_uart(waveshare_cube 0)
pico_add_extra_outputs(waveshare_cube)

# Waveshare official test
add_executable(waveshare_test
    src/waveshare_test.c
    src/ImageData.c
    lib/LCD/LCD_1in28.c
    lib/Config/DEV_Config.c
)

target_include_directories(waveshare_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/LCD
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/Config
)

target_link_libraries(waveshare_test
    pico_stdlib
    hardware_spi
    hardware_i2c
    hardware_gpio
    hardware_pwm
    hardware_adc
)

pico_enable_stdio_usb(waveshare_test 1)
pico_enable_stdio_uart(waveshare_test 0)
pico_add_extra_outputs(waveshare_test)

# Minimal LCD test
add_executable(minimal_lcd
    src/minimal_lcd.c
    lib/LCD/LCD_1in28.c
    lib/Config/DEV_Config.c
)

target_include_directories(minimal_lcd PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/LCD
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/Config
)

target_link_libraries(minimal_lcd
    pico_stdlib
    hardware_spi
    hardware_i2c
    hardware_gpio
    hardware_pwm
    hardware_adc
)

pico_enable_stdio_usb(minimal_lcd 1)
pico_enable_stdio_uart(minimal_lcd 0)
pico_add_extra_outputs(minimal_lcd)

# Rotating cube demo
add_executable(rotating_cube
    src/rotating_cube.c
    lib/LCD/LCD_1in28.c
    lib/Config/DEV_Config.c
)

target_include_directories(rotating_cube PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/LCD
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/Config
)

target_link_libraries(rotating_cube
    pico_stdlib
    hardware_spi
    hardware_i2c
    hardware_gpio
    hardware_pwm
    hardware_adc
)

pico_enable_stdio_usb(rotating_cube 1)
pico_enable_stdio_uart(rotating_cube 0)
pico_add_extra_outputs(rotating_cube)

# Buffered cube demo
add_executable(buffered_cube
    src/buffered_cube.c
    lib/LCD/LCD_1in28.c
    lib/Config/DEV_Config.c
)

target_include_directories(buffered_cube PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/LCD
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/Config
)

target_link_libraries(buffered_cube
    pico_stdlib
    hardware_spi
    hardware_i2c
    hardware_gpio
    hardware_pwm
    hardware_adc
)

pico_enable_stdio_usb(buffered_cube 1)
pico_enable_stdio_uart(buffered_cube 0)
pico_add_extra_outputs(buffered_cube)

# Gravity-locked cube (FINAL)
add_executable(gravity_locked_cube
    src/gravity_locked_cube.c
    lib/LCD/LCD_1in28.c
    lib/Config/DEV_Config.c
)

target_include_directories(gravity_locked_cube PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/LCD
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/Config
)

target_link_libraries(gravity_locked_cube
    pico_stdlib
    hardware_spi
    hardware_i2c
    hardware_gpio
    hardware_pwm
    hardware_adc
)

pico_enable_stdio_usb(gravity_locked_cube 1)
pico_enable_stdio_uart(gravity_locked_cube 0)
pico_add_extra_outputs(gravity_locked_cube)

# Intuitive cube demo
add_executable(intuitive_cube
    src/intuitive_cube.c
    lib/LCD/LCD_1in28.c
    lib/Config/DEV_Config.c
)

target_include_directories(intuitive_cube PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/LCD
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/Config
)

target_link_libraries(intuitive_cube
    pico_stdlib
    hardware_spi
    hardware_i2c
    hardware_gpio
    hardware_pwm
    hardware_adc
)

pico_enable_stdio_usb(intuitive_cube 1)
pico_enable_stdio_uart(intuitive_cube 0)
pico_add_extra_outputs(intuitive_cube)

# Madgwick filter cube demo
add_executable(madgwick_cube
    src/madgwick_cube.c
    lib/LCD/LCD_1in28.c
    lib/Config/DEV_Config.c
)

target_include_directories(madgwick_cube PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/LCD
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/Config
)

target_link_libraries(madgwick_cube
    pico_stdlib
    hardware_spi
    hardware_i2c
    hardware_gpio
    hardware_pwm
    hardware_adc
)

pico_enable_stdio_usb(madgwick_cube 1)
pico_enable_stdio_uart(madgwick_cube 0)
pico_add_extra_outputs(madgwick_cube)

# Axis test
add_executable(axis_test
    src/axis_test.c
    lib/LCD/LCD_1in28.c
    lib/Config/DEV_Config.c
)

target_include_directories(axis_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/LCD
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/Config
)

target_link_libraries(axis_test
    pico_stdlib
    hardware_spi
    hardware_i2c
    hardware_gpio
    hardware_pwm
    hardware_adc
)

pico_enable_stdio_usb(axis_test 1)
pico_enable_stdio_uart(axis_test 0)
pico_add_extra_outputs(axis_test)

# Rotation test
add_executable(rotation_test
    src/rotation_test.c
    lib/LCD/LCD_1in28.c
    lib/Config/DEV_Config.c
)

target_include_directories(rotation_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/LCD
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/Config
)

target_link_libraries(rotation_test
    pico_stdlib
    hardware_spi
    hardware_i2c
    hardware_gpio
    hardware_pwm
    hardware_adc
)

pico_enable_stdio_usb(rotation_test 1)
pico_enable_stdio_uart(rotation_test 0)
pico_add_extra_outputs(rotation_test)

# Configurable cube with CLI
add_executable(configurable_cube
    src/configurable_cube.c
    lib/LCD/LCD_1in28.c
    lib/Config/DEV_Config.c
)

target_include_directories(configurable_cube PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/LCD
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/Config
)

target_link_libraries(configurable_cube
    pico_stdlib
    hardware_spi
    hardware_i2c
    hardware_gpio
    hardware_pwm
    hardware_adc
)

pico_enable_stdio_usb(configurable_cube 1)
pico_enable_stdio_uart(configurable_cube 0)
pico_add_extra_outputs(configurable_cube)

# Copy to Pico when built
# add_custom_command(TARGET rp2350_lcd_demo POST_BUILD
#     COMMAND cp ${CMAKE_CURRENT_BINARY_DIR}/rp2350_lcd_demo.uf2 /media/${USER}/RPI-RP2/
#     COMMENT "Copying to Pico"
#     VERBATIM
# )