#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/unique_id.h"
#include "hardware/watchdog.h"

#include "board_config.h"
#include "system_types.h"
#include "system_init.h"
#include "demo_app.h"
#include "event_handler.h"

// Global system state
system_state_t g_system_state = {0};

static void print_system_info(void) {
    pico_unique_board_id_t board_id;
    pico_get_unique_board_id(&board_id);
    
    printf("\n=== RP2350-LCD-1.28 Demo ===\n");
    printf("Version: %d.%d.%d\n", PROJECT_VERSION_MAJOR, PROJECT_VERSION_MINOR, PROJECT_VERSION_PATCH);
    printf("Board ID: ");
    for (int i = 0; i < PICO_UNIQUE_BOARD_ID_SIZE_BYTES; i++) {
        printf("%02x", board_id.id[i]);
    }
    printf("\n");
    printf("Clock: %d MHz\n", SYSTEM_CLOCK_KHZ / 1000);
    printf("USB: %s\n", g_system_state.usb_connected ? "Connected" : "Disconnected");
    printf("==============================\n\n");
}

static void core1_main(void) {
    // Core 1 handles display updates and UI rendering
    printf("Core 1: Starting display and UI thread\n");
    
    while (1) {
        demo_app_update_display();
        sleep_ms(16);  // ~60 FPS
    }
}

int main(void) {
    system_result_t result;
    
    // Initialize system clock
    set_sys_clock_khz(SYSTEM_CLOCK_KHZ, true);
    
    // Initialize stdio for USB serial
    stdio_init_all();
    
    // Wait for USB connection (optional, remove for standalone operation)
    // while (!stdio_usb_connected()) {
    //     sleep_ms(100);
    // }
    
    // Give some time for USB connection to stabilize
    sleep_ms(2000);
    
    g_system_state.usb_connected = stdio_usb_connected();
    
    print_system_info();
    
    // Initialize all system components
    printf("Initializing system...\n");
    result = system_init();
    if (result != SYSTEM_OK) {
        printf("System initialization failed: %d\n", result);
        return -1;
    }
    
    // Initialize demo application
    printf("Initializing demo application...\n");
    result = demo_app_init();
    if (result != SYSTEM_OK) {
        printf("Demo application initialization failed: %d\n", result);
        return -1;
    }
    
    // Initialize event handler
    printf("Initializing event handler...\n");
    result = event_handler_init();
    if (result != SYSTEM_OK) {
        printf("Event handler initialization failed: %d\n", result);
        return -1;
    }
    
    // Enable watchdog with 8.3 second timeout
    if (watchdog_caused_reboot()) {
        printf("System rebooted by watchdog\n");
    }
    watchdog_enable(WATCHDOG_TIMEOUT_MS, 1);
    
    // Start core 1 for display updates
    printf("Starting core 1...\n");
    multicore_launch_core1(core1_main);
    
    printf("System initialization complete. Starting main loop...\n");
    g_system_state.initialized = true;
    
    uint32_t last_status_print = 0;
    
    // Main event loop on core 0
    while (1) {
        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        g_system_state.uptime_ms = current_time;
        
        // Update watchdog
        watchdog_update();
        
        // Handle events (buttons, IMU, etc.)
        event_handler_process();
        
        // Update demo application logic
        demo_app_update();
        
        // Print status every 5 seconds
        if (current_time - last_status_print > 5000) {
            printf("Uptime: %lu ms, CPU Temp: %.1f°C, VBat: %d mV\n", 
                   g_system_state.uptime_ms,
                   g_system_state.cpu_temperature,
                   g_system_state.vbat_mv);
            last_status_print = current_time;
        }
        
        // Small delay to prevent busy waiting
        sleep_ms(10);
    }
    
    return 0;
}