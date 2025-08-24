# Main source files
add_library(main_src
    system_init.c
    event_handler.c
    demo_app.c
)

target_link_libraries(main_src
    pico_stdlib
    hardware_gpio
    hardware_timer
    display_driver
    sensor_drivers
    system_utils
)