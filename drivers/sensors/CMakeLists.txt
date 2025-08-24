# Sensor driver library
add_library(sensor_drivers
    qmi8658.c
)

target_include_directories(sensor_drivers PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ../../include
)

target_link_libraries(sensor_drivers
    pico_stdlib
    hardware_i2c
    hardware_gpio
    hardware_adc
    hardware_timer
)