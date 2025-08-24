# System utility libraries
add_library(system_utils
    event_queue.c
    state_machine.c
    config_manager.c
    data_logger.c
)

target_include_directories(system_utils PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ../include
)

target_link_libraries(system_utils
    pico_stdlib
    hardware_flash
    hardware_sync
)