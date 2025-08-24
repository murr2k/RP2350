# Test framework for RP2350 project
cmake_minimum_required(VERSION 3.13)

# Include main project include directories
include_directories(
    ../include
    ../drivers/display
    ../drivers/sensors
    ../drivers/common
    ../lib
)

# Test executables (these would run on host for unit testing)
# For embedded testing, these would be separate firmware builds

# Display driver tests
add_executable(test_display
    test_display.c
    ../drivers/display/graphics.c
    mock_hardware.c
)

target_link_libraries(test_display
    # Would link to test framework like Unity or CMock
)

# IMU driver tests  
add_executable(test_imu
    test_imu.c
    mock_hardware.c
)

# Button driver tests
add_executable(test_buttons
    test_buttons.c
    mock_hardware.c
)

# System tests
add_executable(test_system
    test_system.c
    mock_hardware.c
)

# Add test targets
enable_testing()
add_test(NAME display_tests COMMAND test_display)
add_test(NAME imu_tests COMMAND test_imu)  
add_test(NAME button_tests COMMAND test_buttons)
add_test(NAME system_tests COMMAND test_system)