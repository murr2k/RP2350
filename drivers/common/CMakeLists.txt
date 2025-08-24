# Common driver utilities
add_library(common_drivers
    utils.c
    circular_buffer.c
    moving_average.c
)

target_include_directories(common_drivers PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ../../include
)

target_link_libraries(common_drivers
    pico_stdlib
)