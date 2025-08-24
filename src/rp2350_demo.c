/**
 * RP2350-LCD-1.28 Comprehensive Demo Application
 * Showcases all sensors and LCD display capabilities
 */

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

#include "../drivers/sensors/qmi8658.h"
#include "../drivers/display/gc9a01.h"

// Pin definitions for RP2350-LCD-1.28
#define LCD_CS_PIN      9
#define LCD_DC_PIN      8  
#define LCD_RST_PIN     12
#define LCD_BL_PIN      25
#define LCD_SCK_PIN     10
#define LCD_MOSI_PIN    11

#define IMU_SDA_PIN     6
#define IMU_SCL_PIN     7
#define IMU_INT_PIN     23

#define BUTTON_A_PIN    15
#define BUTTON_B_PIN    17

// Demo modes
typedef enum {
    DEMO_IMU_3D,        // 3D cube visualization
    DEMO_IMU_GRAPH,     // Real-time graph
    DEMO_GYRO_COMPASS,  // Gyroscope compass
    DEMO_TILT_BALL,     // Tilt ball game
    DEMO_MOTION_DETECT, // Motion detection
    DEMO_SENSOR_RAW,    // Raw sensor values
    DEMO_MODE_COUNT
} demo_mode_t;

// Global variables
static demo_mode_t current_mode = DEMO_IMU_3D;
static bool mode_changed = true;
static vector3f_t acc_data, gyro_data;
static float temperature = 0.0f;
static uint32_t frame_count = 0;

// 3D cube vertices (normalized)
static const float cube_vertices[][3] = {
    {-1, -1, -1}, {1, -1, -1}, {1, 1, -1}, {-1, 1, -1},  // Back face
    {-1, -1, 1}, {1, -1, 1}, {1, 1, 1}, {-1, 1, 1}        // Front face
};

// Cube edges (vertex pairs)
static const uint8_t cube_edges[][2] = {
    {0, 1}, {1, 2}, {2, 3}, {3, 0},  // Back face
    {4, 5}, {5, 6}, {6, 7}, {7, 4},  // Front face
    {0, 4}, {1, 5}, {2, 6}, {3, 7}   // Connecting edges
};

// Function prototypes
void init_hardware(void);
void core1_display_task(void);
void button_callback(uint gpio, uint32_t events);
void demo_imu_3d(void);
void demo_imu_graph(void);
void demo_gyro_compass(void);
void demo_tilt_ball(void);
void demo_motion_detect(void);
void demo_sensor_raw(void);
void draw_text(uint16_t x, uint16_t y, const char *text, uint16_t color);
void draw_cube(float pitch, float roll, float yaw);
void draw_graph_axes(void);
void update_graph_data(float value, uint16_t color);

// Simple text rendering (using pixel drawing for demo)
void draw_text(uint16_t x, uint16_t y, const char *text, uint16_t color) {
    // Simplified text drawing - in production, use a proper font library
    // For demo purposes, we'll just draw rectangles representing text areas
    size_t len = strlen(text);
    gc9a01_fill_rect(x, y, len * 6, 8, color);
}

// Initialize all hardware
void init_hardware(void) {
    stdio_init_all();
    
    // Initialize ADC for battery monitoring
    adc_init();
    adc_gpio_init(26); // Battery ADC pin
    adc_select_input(0);
    
    // Initialize buttons
    gpio_init(BUTTON_A_PIN);
    gpio_set_dir(BUTTON_A_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_A_PIN);
    gpio_set_irq_enabled_with_callback(BUTTON_A_PIN, GPIO_IRQ_EDGE_FALL, true, &button_callback);
    
    gpio_init(BUTTON_B_PIN);
    gpio_set_dir(BUTTON_B_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_B_PIN);
    gpio_set_irq_enabled(BUTTON_B_PIN, GPIO_IRQ_EDGE_FALL, true);
    
    // Initialize LCD
    if (!gc9a01_init(spi1, LCD_CS_PIN, LCD_DC_PIN, LCD_RST_PIN, 
                     LCD_BL_PIN, LCD_SCK_PIN, LCD_MOSI_PIN)) {
        printf("Failed to initialize LCD!\n");
    }
    
    // Initialize IMU
    if (!qmi8658_init(i2c0, QMI8658_ADDR_LOW, IMU_SDA_PIN, IMU_SCL_PIN)) {
        printf("Failed to initialize IMU!\n");
    }
    
    // Calibrate IMU (optional, takes a few seconds)
    // qmi8658_calibrate(500);
    
    gc9a01_clear(GC9A01_BLACK);
    gc9a01_set_backlight(128); // 50% brightness
}

// Button interrupt handler
void button_callback(uint gpio, uint32_t events) {
    (void)events; // Unused parameter
    if (gpio == BUTTON_A_PIN) {
        current_mode = (current_mode + 1) % DEMO_MODE_COUNT;
        mode_changed = true;
    } else if (gpio == BUTTON_B_PIN) {
        // Reset/calibrate IMU
        qmi8658_calibrate(100);
    }
}

// Draw 3D cube with rotation
void draw_cube(float pitch, float roll, float yaw) {
    // Convert angles to radians
    float p = pitch * M_PI / 180.0f;
    float r = roll * M_PI / 180.0f;
    float y = yaw * M_PI / 180.0f;
    
    // Rotation matrices
    float cos_p = cosf(p), sin_p = sinf(p);
    float cos_r = cosf(r), sin_r = sinf(r);
    float cos_y = cosf(y), sin_y = sinf(y);
    
    // Project and transform vertices
    int16_t projected[8][2];
    
    for (int i = 0; i < 8; i++) {
        // Apply rotations
        float x = cube_vertices[i][0];
        float y = cube_vertices[i][1];
        float z = cube_vertices[i][2];
        
        // Rotate around Y axis (yaw)
        float x1 = x * cos_y - z * sin_y;
        float z1 = x * sin_y + z * cos_y;
        
        // Rotate around X axis (pitch)
        float y2 = y * cos_p - z1 * sin_p;
        float z2 = y * sin_p + z1 * cos_p;
        
        // Rotate around Z axis (roll)
        float x3 = x1 * cos_r - y2 * sin_r;
        float y3 = x1 * sin_r + y2 * cos_r;
        
        // Project to 2D (simple perspective)
        float scale = 40.0f / (4.0f + z2);
        projected[i][0] = (int16_t)(x3 * scale + 120);
        projected[i][1] = (int16_t)(y3 * scale + 120);
    }
    
    // Draw edges
    for (int i = 0; i < 12; i++) {
        uint8_t v1 = cube_edges[i][0];
        uint8_t v2 = cube_edges[i][1];
        
        // Choose color based on edge type
        uint16_t color = GC9A01_GREEN;
        if (i < 4) color = GC9A01_RED;      // Back face
        else if (i < 8) color = GC9A01_BLUE; // Front face
        
        gc9a01_draw_line(projected[v1][0], projected[v1][1],
                        projected[v2][0], projected[v2][1], color);
    }
}

// Demo: 3D IMU visualization
void demo_imu_3d(void) {
    if (mode_changed) {
        gc9a01_clear(GC9A01_BLACK);
        draw_text(80, 10, "3D IMU DEMO", GC9A01_WHITE);
        mode_changed = false;
    }
    
    // Clear cube area
    gc9a01_fill_rect(40, 40, 160, 160, GC9A01_BLACK);
    
    // Calculate rotation angles from accelerometer
    float pitch = atan2f(acc_data.x, sqrtf(acc_data.y * acc_data.y + acc_data.z * acc_data.z)) * 180.0f / M_PI;
    float roll = atan2f(acc_data.y, acc_data.z) * 180.0f / M_PI;
    float yaw = frame_count * 2.0f; // Rotate continuously for demo
    
    // Draw the cube
    draw_cube(pitch, roll, yaw);
    
    // Display values
    char buf[32];
    sprintf(buf, "P:%.1f R:%.1f", pitch, roll);
    draw_text(70, 210, buf, GC9A01_YELLOW);
}

// Demo: Real-time sensor graph
void demo_imu_graph(void) {
    // static uint16_t graph_x = 40; // Reserved for future use
    static float history[160] = {0};
    static int history_idx = 0;
    
    if (mode_changed) {
        gc9a01_clear(GC9A01_BLACK);
        draw_text(70, 10, "SENSOR GRAPH", GC9A01_WHITE);
        draw_graph_axes();
        mode_changed = false;
        // graph_x = 40; // Reserved for future use
        memset(history, 0, sizeof(history));
        history_idx = 0;
    }
    
    // Add new data point
    history[history_idx] = acc_data.z;
    history_idx = (history_idx + 1) % 160;
    
    // Clear graph area
    gc9a01_fill_rect(40, 60, 160, 120, GC9A01_BLACK);
    
    // Draw graph lines
    for (int i = 0; i < 159; i++) {
        int idx1 = (history_idx + i) % 160;
        int idx2 = (history_idx + i + 1) % 160;
        
        int y1 = 120 - (int)(history[idx1] * 40 + 60);
        int y2 = 120 - (int)(history[idx2] * 40 + 60);
        
        gc9a01_draw_line(40 + i, y1, 41 + i, y2, GC9A01_GREEN);
    }
    
    // Draw axes
    draw_graph_axes();
}

void draw_graph_axes(void) {
    // X axis
    gc9a01_draw_line(40, 120, 200, 120, GC9A01_WHITE);
    // Y axis
    gc9a01_draw_line(40, 60, 40, 180, GC9A01_WHITE);
    
    // Grid lines
    for (int i = 0; i < 4; i++) {
        gc9a01_draw_line(40, 60 + i * 30, 200, 60 + i * 30, GC9A01_GRAY);
    }
}

// Demo: Gyroscope compass
void demo_gyro_compass(void) {
    static float heading = 0.0f;
    
    if (mode_changed) {
        gc9a01_clear(GC9A01_BLACK);
        draw_text(65, 10, "GYRO COMPASS", GC9A01_WHITE);
        mode_changed = false;
        heading = 0.0f;
    }
    
    // Update heading based on gyro Z axis
    heading += gyro_data.z * 0.01f; // Integration
    if (heading < 0) heading += 360.0f;
    if (heading >= 360) heading -= 360.0f;
    
    // Clear compass area
    gc9a01_fill_rect(40, 40, 160, 160, GC9A01_BLACK);
    
    // Draw compass circle
    gc9a01_draw_circle(120, 120, 70, GC9A01_WHITE);
    
    // Draw cardinal directions
    draw_text(115, 40, "N", GC9A01_RED);
    draw_text(115, 190, "S", GC9A01_WHITE);
    draw_text(40, 115, "W", GC9A01_WHITE);
    draw_text(190, 115, "E", GC9A01_WHITE);
    
    // Draw compass needle
    float angle = (heading - 90) * M_PI / 180.0f;
    int x = (int)(cosf(angle) * 60) + 120;
    int y = (int)(sinf(angle) * 60) + 120;
    
    gc9a01_draw_line(120, 120, x, y, GC9A01_RED);
    gc9a01_fill_circle(x, y, 5, GC9A01_RED);
    
    // Display heading
    char buf[32];
    sprintf(buf, "%.1f deg", heading);
    draw_text(85, 210, buf, GC9A01_YELLOW);
}

// Demo: Tilt ball game
void demo_tilt_ball(void) {
    static float ball_x = 120.0f;
    static float ball_y = 120.0f;
    static float ball_vx = 0.0f;
    static float ball_vy = 0.0f;
    
    if (mode_changed) {
        gc9a01_clear(GC9A01_BLACK);
        draw_text(75, 10, "TILT BALL", GC9A01_WHITE);
        gc9a01_draw_circle(120, 120, 100, GC9A01_WHITE);
        mode_changed = false;
        ball_x = 120.0f;
        ball_y = 120.0f;
        ball_vx = 0.0f;
        ball_vy = 0.0f;
    }
    
    // Clear old ball position
    gc9a01_fill_circle((uint16_t)ball_x, (uint16_t)ball_y, 8, GC9A01_BLACK);
    
    // Update ball physics based on accelerometer
    ball_vx += acc_data.x * 0.5f;
    ball_vy -= acc_data.y * 0.5f;
    
    // Apply friction
    ball_vx *= 0.95f;
    ball_vy *= 0.95f;
    
    // Update position
    ball_x += ball_vx;
    ball_y += ball_vy;
    
    // Keep ball within circle
    float dist = sqrtf((ball_x - 120) * (ball_x - 120) + (ball_y - 120) * (ball_y - 120));
    if (dist > 92) {
        float angle = atan2f(ball_y - 120, ball_x - 120);
        ball_x = 120 + cosf(angle) * 92;
        ball_y = 120 + sinf(angle) * 92;
        
        // Bounce
        ball_vx *= -0.5f;
        ball_vy *= -0.5f;
    }
    
    // Draw new ball position
    gc9a01_fill_circle((uint16_t)ball_x, (uint16_t)ball_y, 8, GC9A01_GREEN);
    
    // Redraw circle boundary
    gc9a01_draw_circle(120, 120, 100, GC9A01_WHITE);
}

// Demo: Motion detection
void demo_motion_detect(void) {
    static float motion_threshold = 0.5f;
    static uint32_t motion_count = 0;
    static bool in_motion = false;
    
    if (mode_changed) {
        gc9a01_clear(GC9A01_BLACK);
        draw_text(60, 10, "MOTION DETECT", GC9A01_WHITE);
        mode_changed = false;
        motion_count = 0;
    }
    
    // Calculate total acceleration magnitude
    float acc_mag = sqrtf(acc_data.x * acc_data.x + 
                         acc_data.y * acc_data.y + 
                         acc_data.z * acc_data.z);
    
    // Check for motion (deviation from 1g)
    bool motion_detected = fabsf(acc_mag - 1.0f) > motion_threshold ||
                          fabsf(gyro_data.x) > 30.0f ||
                          fabsf(gyro_data.y) > 30.0f ||
                          fabsf(gyro_data.z) > 30.0f;
    
    if (motion_detected && !in_motion) {
        motion_count++;
        in_motion = true;
    } else if (!motion_detected) {
        in_motion = false;
    }
    
    // Display status
    uint16_t status_color = motion_detected ? GC9A01_RED : GC9A01_GREEN;
    gc9a01_fill_circle(120, 100, 40, status_color);
    
    if (motion_detected) {
        draw_text(85, 95, "MOTION!", GC9A01_WHITE);
    } else {
        draw_text(95, 95, "STILL", GC9A01_BLACK);
    }
    
    // Display count
    char buf[32];
    sprintf(buf, "Count: %lu", motion_count);
    draw_text(75, 160, buf, GC9A01_YELLOW);
    
    // Display sensitivity
    sprintf(buf, "Sens: %.1f", motion_threshold);
    draw_text(75, 180, buf, GC9A01_CYAN);
}

// Demo: Raw sensor values
void demo_sensor_raw(void) {
    if (mode_changed) {
        gc9a01_clear(GC9A01_BLACK);
        draw_text(70, 10, "RAW SENSORS", GC9A01_WHITE);
        mode_changed = false;
    }
    
    // Clear data area
    gc9a01_fill_rect(20, 40, 200, 160, GC9A01_BLACK);
    
    // Display accelerometer data
    draw_text(20, 50, "ACCELEROMETER", GC9A01_CYAN);
    char buf[64];
    sprintf(buf, "X: %.3f g", acc_data.x);
    draw_text(20, 70, buf, GC9A01_WHITE);
    sprintf(buf, "Y: %.3f g", acc_data.y);
    draw_text(20, 85, buf, GC9A01_WHITE);
    sprintf(buf, "Z: %.3f g", acc_data.z);
    draw_text(20, 100, buf, GC9A01_WHITE);
    
    // Display gyroscope data
    draw_text(20, 120, "GYROSCOPE", GC9A01_CYAN);
    sprintf(buf, "X: %.1f dps", gyro_data.x);
    draw_text(20, 140, buf, GC9A01_WHITE);
    sprintf(buf, "Y: %.1f dps", gyro_data.y);
    draw_text(20, 155, buf, GC9A01_WHITE);
    sprintf(buf, "Z: %.1f dps", gyro_data.z);
    draw_text(20, 170, buf, GC9A01_WHITE);
    
    // Display temperature
    sprintf(buf, "Temp: %.1f C", temperature);
    draw_text(20, 190, buf, GC9A01_YELLOW);
    
    // Display battery voltage (if available)
    uint16_t adc_raw = adc_read();
    float battery_v = adc_raw * 3.3f * 2.0f / 4095.0f; // Assuming voltage divider
    sprintf(buf, "Batt: %.2f V", battery_v);
    draw_text(20, 205, buf, GC9A01_GREEN);
}

// Core 1: Display update task
void core1_display_task(void) {
    while (1) {
        switch (current_mode) {
            case DEMO_IMU_3D:
                demo_imu_3d();
                break;
            case DEMO_IMU_GRAPH:
                demo_imu_graph();
                break;
            case DEMO_GYRO_COMPASS:
                demo_gyro_compass();
                break;
            case DEMO_TILT_BALL:
                demo_tilt_ball();
                break;
            case DEMO_MOTION_DETECT:
                demo_motion_detect();
                break;
            case DEMO_SENSOR_RAW:
                demo_sensor_raw();
                break;
            case DEMO_MODE_COUNT:
                // Should never reach here
                break;
        }
        
        frame_count++;
        sleep_ms(20); // ~50 FPS
    }
}

// Main function
int main(void) {
    // Initialize hardware
    init_hardware();
    
    printf("\n=== RP2350-LCD-1.28 Demo ===\n");
    printf("Button A: Switch demo mode\n");
    printf("Button B: Calibrate IMU\n");
    printf("===========================\n\n");
    
    // Start display task on Core 1
    multicore_launch_core1(core1_display_task);
    
    // Main loop on Core 0: Read sensors
    while (1) {
        // Read IMU data
        if (qmi8658_data_ready()) {
            qmi8658_read(&acc_data, &gyro_data);
            temperature = qmi8658_read_temperature();
        }
        
        // Print debug info occasionally
        static uint32_t print_counter = 0;
        if (++print_counter >= 50) { // Every second at 50Hz
            printf("Mode: %d, Acc: %.2f,%.2f,%.2f, Gyro: %.1f,%.1f,%.1f, Temp: %.1f\n",
                   current_mode,
                   acc_data.x, acc_data.y, acc_data.z,
                   gyro_data.x, gyro_data.y, gyro_data.z,
                   temperature);
            print_counter = 0;
        }
        
        sleep_ms(20); // 50Hz sensor reading
    }
    
    return 0;
}