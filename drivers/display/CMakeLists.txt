# Display driver library
add_library(display_driver
    gc9a01.c
)

target_include_directories(display_driver PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ../../include
)

target_link_libraries(display_driver
    pico_stdlib
    hardware_spi
    hardware_gpio
    hardware_pwm
)