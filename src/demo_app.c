#include "demo_app.h"
#include "display_driver.h"
#include "imu_driver.h"
#include "button_driver.h"
#include "adc_driver.h"
#include "system_types.h"
#include "board_config.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

// Demo application state
typedef enum {
    DEMO_STATE_SPLASH,
    DEMO_STATE_MAIN_MENU,
    DEMO_STATE_IMU_DEMO,
    DEMO_STATE_BUTTON_TEST,
    DEMO_STATE_SENSOR_MONITOR,
    DEMO_STATE_GRAPHICS_DEMO
} demo_state_t;

typedef struct {
    demo_state_t current_state;
    demo_state_t previous_state;
    uint32_t state_enter_time;
    uint32_t frame_counter;
    bool needs_redraw;
    
    // Menu selection
    uint8_t menu_selection;
    uint8_t menu_items;
    
    // IMU demo data
    imu_data_t imu_data;
    float roll, pitch, yaw;
    
    // Graphics demo variables
    float animation_phase;
    uint16_t graphics_color;
    
} demo_app_state_t;

static demo_app_state_t app_state = {0};

// Menu items
static const char* menu_items[] = {
    "IMU Demo",
    "Button Test", 
    "Sensor Monitor",
    "Graphics Demo"
};

static void change_state(demo_state_t new_state) {
    app_state.previous_state = app_state.current_state;
    app_state.current_state = new_state;
    app_state.state_enter_time = to_ms_since_boot(get_absolute_time());
    app_state.needs_redraw = true;
    
    DEBUG_PRINT("State change: %d -> %d", app_state.previous_state, new_state);
}

static void draw_splash_screen(void) {
    display_clear(COLOR_BLACK);
    
    // Draw title
    display_draw_text("RP2350-LCD-1.28", 30, 60, COLOR_WHITE, COLOR_BLACK, FONT_SIZE_16);
    display_draw_text("Demo Application", 35, 85, COLOR_CYAN, COLOR_BLACK, FONT_SIZE_12);
    
    // Draw version
    display_draw_text("v1.0.0", 90, 110, COLOR_GRAY, COLOR_BLACK, FONT_SIZE_8);
    
    // Draw loading animation
    uint32_t time_in_state = to_ms_since_boot(get_absolute_time()) - app_state.state_enter_time;
    uint8_t dots = (time_in_state / 300) % 4;
    char loading_text[20] = "Loading";
    for (int i = 0; i < dots; i++) {
        strcat(loading_text, ".");
    }
    display_draw_text(loading_text, 85, 140, COLOR_GREEN, COLOR_BLACK, FONT_SIZE_12);
    
    // Draw circle animation
    float angle = (time_in_state / 50.0f) * 0.1f;
    uint16_t center_x = 120;
    uint16_t center_y = 170;
    uint16_t radius = 20;
    
    for (int i = 0; i < 8; i++) {
        float a = angle + (i * M_PI / 4.0f);
        uint16_t x = center_x + (uint16_t)(cos(a) * radius);
        uint16_t y = center_y + (uint16_t)(sin(a) * radius);
        color565_t color = (i < 4) ? COLOR_BLUE : COLOR_DARK_GRAY;
        display_fill_circle(x, y, 3, color);
    }
    
    // Auto-advance after 3 seconds
    if (time_in_state > 3000) {
        change_state(DEMO_STATE_MAIN_MENU);
    }
}

static void draw_main_menu(void) {
    display_clear(COLOR_BLACK);
    
    // Draw title
    display_draw_text("Main Menu", 70, 20, COLOR_WHITE, COLOR_BLACK, FONT_SIZE_16);
    
    // Draw menu items
    app_state.menu_items = sizeof(menu_items) / sizeof(menu_items[0]);
    
    for (int i = 0; i < app_state.menu_items; i++) {
        color565_t text_color = (i == app_state.menu_selection) ? COLOR_YELLOW : COLOR_WHITE;
        color565_t bg_color = (i == app_state.menu_selection) ? COLOR_BLUE : COLOR_BLACK;
        
        if (i == app_state.menu_selection) {
            display_fill_rect(20, 60 + i * 25, 200, 20, bg_color);
        }
        
        display_draw_text(menu_items[i], 30, 65 + i * 25, text_color, bg_color, FONT_SIZE_12);
    }
    
    // Draw instructions
    display_draw_text("A/B: Select  X: Enter", 30, 200, COLOR_GRAY, COLOR_BLACK, FONT_SIZE_8);
    display_draw_text("Y: Back", 30, 215, COLOR_GRAY, COLOR_BLACK, FONT_SIZE_8);
}

static void draw_imu_demo(void) {
    display_clear(COLOR_BLACK);
    
    // Draw title
    display_draw_text("IMU Demo", 80, 10, COLOR_WHITE, COLOR_BLACK, FONT_SIZE_16);
    
    // Draw accelerometer data
    char text_buffer[50];
    sprintf(text_buffer, "Accel X: %6.2f", app_state.imu_data.accelerometer.x);
    display_draw_text(text_buffer, 10, 40, COLOR_GREEN, COLOR_BLACK, FONT_SIZE_8);
    
    sprintf(text_buffer, "Accel Y: %6.2f", app_state.imu_data.accelerometer.y);
    display_draw_text(text_buffer, 10, 55, COLOR_GREEN, COLOR_BLACK, FONT_SIZE_8);
    
    sprintf(text_buffer, "Accel Z: %6.2f", app_state.imu_data.accelerometer.z);
    display_draw_text(text_buffer, 10, 70, COLOR_GREEN, COLOR_BLACK, FONT_SIZE_8);
    
    // Draw gyroscope data
    sprintf(text_buffer, "Gyro X:  %6.2f", app_state.imu_data.gyroscope.x);
    display_draw_text(text_buffer, 10, 90, COLOR_CYAN, COLOR_BLACK, FONT_SIZE_8);
    
    sprintf(text_buffer, "Gyro Y:  %6.2f", app_state.imu_data.gyroscope.y);
    display_draw_text(text_buffer, 10, 105, COLOR_CYAN, COLOR_BLACK, FONT_SIZE_8);
    
    sprintf(text_buffer, "Gyro Z:  %6.2f", app_state.imu_data.gyroscope.z);
    display_draw_text(text_buffer, 10, 120, COLOR_CYAN, COLOR_BLACK, FONT_SIZE_8);
    
    // Draw temperature
    sprintf(text_buffer, "Temp: %.1f C", app_state.imu_data.temperature);
    display_draw_text(text_buffer, 10, 140, COLOR_ORANGE, COLOR_BLACK, FONT_SIZE_8);
    
    // Draw orientation visualization (simple bubble level)
    uint16_t center_x = 180;
    uint16_t center_y = 100;
    uint16_t bubble_radius = 30;
    
    display_draw_circle(center_x, center_y, bubble_radius, COLOR_WHITE);
    
    // Calculate bubble position based on accelerometer
    float tilt_x = CLAMP(app_state.imu_data.accelerometer.x / 9.81f, -1.0f, 1.0f);
    float tilt_y = CLAMP(app_state.imu_data.accelerometer.y / 9.81f, -1.0f, 1.0f);
    
    uint16_t bubble_x = center_x + (int16_t)(tilt_x * (bubble_radius - 5));
    uint16_t bubble_y = center_y - (int16_t)(tilt_y * (bubble_radius - 5));
    
    display_fill_circle(bubble_x, bubble_y, 5, COLOR_GREEN);
    
    display_draw_text("Y: Back to Menu", 10, 220, COLOR_GRAY, COLOR_BLACK, FONT_SIZE_8);
}

static void draw_button_test(void) {
    display_clear(COLOR_BLACK);
    
    display_draw_text("Button Test", 70, 10, COLOR_WHITE, COLOR_BLACK, FONT_SIZE_16);
    
    button_states_t button_states;
    button_get_all_states(&button_states);
    
    // Draw button states
    const char* button_names[] = {"A", "B", "X", "Y"};
    color565_t button_colors[] = {COLOR_RED, COLOR_GREEN, COLOR_BLUE, COLOR_YELLOW};
    
    for (int i = 0; i < 4; i++) {
        button_state_t state = (&button_states.a)[i];
        
        uint16_t x = 60 + (i % 2) * 100;
        uint16_t y = 60 + (i / 2) * 80;
        
        // Draw button representation
        color565_t color = (state == BUTTON_PRESSED || state == BUTTON_HELD) ? 
                          button_colors[i] : COLOR_DARK_GRAY;
        
        display_fill_rect(x, y, 60, 40, color);
        display_draw_rect(x, y, 60, 40, COLOR_WHITE);
        
        // Draw button label
        display_draw_text(button_names[i], x + 25, y + 15, COLOR_BLACK, color, FONT_SIZE_16);
        
        // Draw state text
        const char* state_text = (state == BUTTON_PRESSED) ? "PRESS" :
                                (state == BUTTON_HELD) ? "HELD" : "FREE";
        display_draw_text(state_text, x + 10, y + 45, COLOR_WHITE, COLOR_BLACK, FONT_SIZE_8);
    }
    
    display_draw_text("Press buttons to test", 50, 200, COLOR_GRAY, COLOR_BLACK, FONT_SIZE_8);
    display_draw_text("Y: Back to Menu", 10, 220, COLOR_GRAY, COLOR_BLACK, FONT_SIZE_8);
}

static void draw_sensor_monitor(void) {
    display_clear(COLOR_BLACK);
    
    display_draw_text("Sensor Monitor", 60, 10, COLOR_WHITE, COLOR_BLACK, FONT_SIZE_16);
    
    // Read and display sensor values
    uint16_t vbat_mv;
    float cpu_temp;
    uint8_t light_level;
    
    if (adc_read_battery_voltage(&vbat_mv) == SYSTEM_OK) {
        char text[30];
        sprintf(text, "Battery: %d mV", vbat_mv);
        display_draw_text(text, 10, 40, COLOR_GREEN, COLOR_BLACK, FONT_SIZE_12);
    }
    
    if (adc_read_internal_temperature(&cpu_temp) == SYSTEM_OK) {
        char text[30];
        sprintf(text, "CPU Temp: %.1f C", cpu_temp);
        display_draw_text(text, 10, 60, COLOR_ORANGE, COLOR_BLACK, FONT_SIZE_12);
    }
    
    if (adc_read_light_sensor(&light_level) == SYSTEM_OK) {
        char text[30];
        sprintf(text, "Light: %d%%", light_level);
        display_draw_text(text, 10, 80, COLOR_YELLOW, COLOR_BLACK, FONT_SIZE_12);
        
        // Draw light level bar
        display_draw_rect(10, 95, 200, 20, COLOR_WHITE);
        uint16_t bar_width = (light_level * 198) / 100;
        display_fill_rect(11, 96, bar_width, 18, COLOR_YELLOW);
    }
    
    // Display system uptime
    extern system_state_t g_system_state;
    char uptime_text[30];
    uint32_t uptime_sec = g_system_state.uptime_ms / 1000;
    sprintf(uptime_text, "Uptime: %lu:%02lu", uptime_sec / 60, uptime_sec % 60);
    display_draw_text(uptime_text, 10, 130, COLOR_CYAN, COLOR_BLACK, FONT_SIZE_12);
    
    // Display USB connection status
    const char* usb_status = g_system_state.usb_connected ? "Connected" : "Disconnected";
    color565_t usb_color = g_system_state.usb_connected ? COLOR_GREEN : COLOR_RED;
    display_draw_text("USB: ", 10, 150, COLOR_WHITE, COLOR_BLACK, FONT_SIZE_12);
    display_draw_text(usb_status, 50, 150, usb_color, COLOR_BLACK, FONT_SIZE_12);
    
    display_draw_text("Y: Back to Menu", 10, 220, COLOR_GRAY, COLOR_BLACK, FONT_SIZE_8);
}

static void draw_graphics_demo(void) {
    display_clear(COLOR_BLACK);
    
    display_draw_text("Graphics Demo", 65, 10, COLOR_WHITE, COLOR_BLACK, FONT_SIZE_16);
    
    // Animated graphics
    uint32_t time_in_state = to_ms_since_boot(get_absolute_time()) - app_state.state_enter_time;
    app_state.animation_phase = (time_in_state / 20.0f) * 0.01f;
    
    // Rotating circles
    uint16_t center_x = 120;
    uint16_t center_y = 120;
    
    for (int i = 0; i < 6; i++) {
        float angle = app_state.animation_phase + (i * M_PI / 3.0f);
        uint16_t x = center_x + (uint16_t)(cos(angle) * 40);
        uint16_t y = center_y + (uint16_t)(sin(angle) * 40);
        
        color565_t color = RGB_TO_RGB565(
            (uint8_t)(127 + 127 * cos(angle)),
            (uint8_t)(127 + 127 * sin(angle)),
            (uint8_t)(127 + 127 * cos(angle + M_PI/2))
        );
        
        display_fill_circle(x, y, 8, color);
    }
    
    // Color-changing border
    app_state.graphics_color = RGB_TO_RGB565(
        (uint8_t)(127 + 127 * cos(app_state.animation_phase)),
        (uint8_t)(127 + 127 * cos(app_state.animation_phase + 2*M_PI/3)),
        (uint8_t)(127 + 127 * cos(app_state.animation_phase + 4*M_PI/3))
    );
    
    for (int i = 0; i < 4; i++) {
        display_draw_rect(i, i, 240-2*i, 240-2*i, app_state.graphics_color);
    }
    
    display_draw_text("Y: Back to Menu", 10, 220, COLOR_GRAY, COLOR_BLACK, FONT_SIZE_8);
}

system_result_t demo_app_init(void) {
    memset(&app_state, 0, sizeof(app_state));
    
    app_state.current_state = DEMO_STATE_SPLASH;
    app_state.state_enter_time = to_ms_since_boot(get_absolute_time());
    app_state.needs_redraw = true;
    
    printf("Demo application initialized\n");
    return SYSTEM_OK;
}

system_result_t demo_app_update(void) {
    // Update button states
    button_update();
    
    // Handle button input based on current state
    switch (app_state.current_state) {
        case DEMO_STATE_MAIN_MENU:
            if (button_was_pressed(BUTTON_A)) {
                app_state.menu_selection = (app_state.menu_selection + 1) % app_state.menu_items;
                app_state.needs_redraw = true;
            }
            if (button_was_pressed(BUTTON_B)) {
                app_state.menu_selection = (app_state.menu_selection + app_state.menu_items - 1) % app_state.menu_items;
                app_state.needs_redraw = true;
            }
            if (button_was_pressed(BUTTON_X)) {
                demo_state_t new_state = DEMO_STATE_IMU_DEMO + app_state.menu_selection;
                change_state(new_state);
            }
            break;
            
        case DEMO_STATE_IMU_DEMO:
        case DEMO_STATE_BUTTON_TEST:
        case DEMO_STATE_SENSOR_MONITOR:
        case DEMO_STATE_GRAPHICS_DEMO:
            if (button_was_pressed(BUTTON_Y)) {
                change_state(DEMO_STATE_MAIN_MENU);
            }
            // These states need continuous updates
            app_state.needs_redraw = true;
            break;
            
        default:
            break;
    }
    
    // Update IMU data if in IMU demo
    if (app_state.current_state == DEMO_STATE_IMU_DEMO) {
        imu_read_data(&app_state.imu_data);
    }
    
    // Update ADC readings
    adc_update_all();
    
    return SYSTEM_OK;
}

void demo_app_update_display(void) {
    if (!app_state.needs_redraw) {
        return;
    }
    
    switch (app_state.current_state) {
        case DEMO_STATE_SPLASH:
            draw_splash_screen();
            break;
        case DEMO_STATE_MAIN_MENU:
            draw_main_menu();
            app_state.needs_redraw = false;  // Only redraw on input
            break;
        case DEMO_STATE_IMU_DEMO:
            draw_imu_demo();
            break;
        case DEMO_STATE_BUTTON_TEST:
            draw_button_test();
            break;
        case DEMO_STATE_SENSOR_MONITOR:
            draw_sensor_monitor();
            break;
        case DEMO_STATE_GRAPHICS_DEMO:
            draw_graphics_demo();
            break;
    }
    
    display_update();
    app_state.frame_counter++;
}