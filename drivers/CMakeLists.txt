# Add driver subdirectories
add_subdirectory(display)
add_subdirectory(sensors)
add_subdirectory(common)

# Create combined driver library
add_library(drivers_combined INTERFACE)

target_link_libraries(drivers_combined INTERFACE
    display_driver
    sensor_drivers
    common_drivers
)