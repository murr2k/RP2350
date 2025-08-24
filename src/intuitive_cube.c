/**
 * Intuitive Gravity Cube
 * Cube tilts to match device tilt - more intuitive visual feedback
 */

#include "pico/stdlib.h"
#include "LCD_1in28.h"
#include "DEV_Config.h"
#include "hardware/i2c.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define PI 3.14159265359f
#define WIDTH 240
#define HEIGHT 240

// I2C pins for IMU
#define I2C_SDA_PIN     6
#define I2C_SCL_PIN     7

// QMI8658 I2C address and registers
#define QMI8658_ADDR    0x6B
#define QMI8658_WHO_AM_I        0x00
#define QMI8658_CTRL1           0x02
#define QMI8658_CTRL2           0x03
#define QMI8658_CTRL3           0x04
#define QMI8658_CTRL7           0x08
#define QMI8658_AX_L            0x35

// Frame buffer
static uint16_t frame_buffer[WIDTH * HEIGHT];

// Cube vertices
typedef struct {
    float x, y, z;
} vertex_t;

vertex_t cube[8] = {
    {-50, -50, -50}, {50, -50, -50}, {50, 50, -50}, {-50, 50, -50},
    {-50, -50,  50}, {50, -50,  50}, {50, 50,  50}, {-50, 50,  50}
};

const int edges[12][2] = {
    {0,1}, {1,2}, {2,3}, {3,0},  // Back face
    {4,5}, {5,6}, {6,7}, {7,4},  // Front face
    {0,4}, {1,5}, {2,6}, {3,7}   // Connecting edges
};

// I2C functions
static void i2c_write_register(uint8_t addr, uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    i2c_write_blocking(i2c1, addr, buf, 2, false);
}

static uint8_t i2c_read_register(uint8_t addr, uint8_t reg) {
    uint8_t value;
    i2c_write_blocking(i2c1, addr, &reg, 1, true);
    i2c_read_blocking(i2c1, addr, &value, 1, false);
    return value;
}

static int16_t i2c_read_16bit(uint8_t addr, uint8_t reg_l) {
    uint8_t low = i2c_read_register(addr, reg_l);
    uint8_t high = i2c_read_register(addr, reg_l + 1);
    return (int16_t)((high << 8) | low);
}

// Initialize QMI8658
static bool qmi8658_init(void) {
    uint8_t who_am_i = i2c_read_register(QMI8658_ADDR, QMI8658_WHO_AM_I);
    printf("QMI8658 WHO_AM_I: 0x%02X\n", who_am_i);
    
    if (who_am_i != 0x05) {
        return false;
    }
    
    // Reset
    i2c_write_register(QMI8658_ADDR, QMI8658_CTRL1, 0x60);
    sleep_ms(10);
    
    // Configure accelerometer: 2g range, 250Hz
    i2c_write_register(QMI8658_ADDR, QMI8658_CTRL2, 0x04);
    
    // Configure gyroscope: 256 dps range, 250Hz
    i2c_write_register(QMI8658_ADDR, QMI8658_CTRL3, 0x54);
    
    // Enable sensors
    i2c_write_register(QMI8658_ADDR, QMI8658_CTRL7, 0x03);
    
    return true;
}

// Read accelerometer data
static void read_accel(float *ax, float *ay, float *az) {
    int16_t raw_ax = i2c_read_16bit(QMI8658_ADDR, QMI8658_AX_L);
    int16_t raw_ay = i2c_read_16bit(QMI8658_ADDR, QMI8658_AX_L + 2);
    int16_t raw_az = i2c_read_16bit(QMI8658_ADDR, QMI8658_AX_L + 4);
    
    // Convert to g's (2g range)
    *ax = raw_ax / 16384.0f;
    *ay = raw_ay / 16384.0f;
    *az = raw_az / 16384.0f;
}

// Frame buffer functions
void set_pixel(int x, int y, uint16_t color) {
    if (x >= 0 && x < WIDTH && y >= 0 && y < HEIGHT) {
        frame_buffer[y * WIDTH + x] = ((color << 8) & 0xFF00) | (color >> 8);
    }
}

void draw_line(int x0, int y0, int x1, int y1, uint16_t color) {
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    
    while (1) {
        set_pixel(x0, y0, color);
        if (x0 == x1 && y0 == y1) break;
        
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }
}

void clear_buffer(uint16_t color) {
    uint16_t swapped = ((color << 8) & 0xFF00) | (color >> 8);
    for (int i = 0; i < WIDTH * HEIGHT; i++) {
        frame_buffer[i] = swapped;
    }
}

void display_buffer(void) {
    LCD_1IN28_Display(frame_buffer);
}

// 3D rotation functions
void rotate_x(vertex_t *v, float angle) {
    float cos_a = cosf(angle);
    float sin_a = sinf(angle);
    float y = v->y;
    float z = v->z;
    v->y = y * cos_a - z * sin_a;
    v->z = y * sin_a + z * cos_a;
}

void rotate_y(vertex_t *v, float angle) {
    float cos_a = cosf(angle);
    float sin_a = sinf(angle);
    float x = v->x;
    float z = v->z;
    v->x = x * cos_a + z * sin_a;
    v->z = -x * sin_a + z * cos_a;
}

// Project 3D to 2D
void project(vertex_t v, int *x, int *y) {
    float distance = 200.0f;
    float z = v.z + distance;
    *x = (int)(120 + (v.x * 120) / z);
    *y = (int)(120 + (v.y * 120) / z);
}

// Draw wireframe cube
void draw_cube(vertex_t *vertices) {
    int screen[8][2];
    
    // Project all vertices
    for (int i = 0; i < 8; i++) {
        project(vertices[i], &screen[i][0], &screen[i][1]);
    }
    
    // Draw edges with depth-based colors
    for (int i = 0; i < 12; i++) {
        int v0 = edges[i][0];
        int v1 = edges[i][1];
        
        // Color based on edge group
        uint16_t color = 0xFFFF;  // White default
        if (i < 4) color = 0xF800;      // Red - back face
        else if (i < 8) color = 0x07E0; // Green - front face
        else color = 0x001F;             // Blue - connecting edges
        
        draw_line(screen[v0][0], screen[v0][1], 
                 screen[v1][0], screen[v1][1], color);
    }
}

// Draw horizon line
void draw_horizon(float tilt_x, float tilt_y) {
    // Draw horizontal reference line
    draw_line(20, 120, 220, 120, 0x0410);  // Dark green
    
    // Draw vertical reference line  
    draw_line(120, 20, 120, 220, 0x0410);  // Dark green
    
    // Draw tilt indicators
    int tx = 120 + (int)(tilt_y * 100);  // Left-right tilt
    int ty = 120 - (int)(tilt_x * 100);  // Forward-back tilt
    
    // Clamp to screen
    if (tx < 20) tx = 20;
    if (tx > 220) tx = 220;
    if (ty < 20) ty = 20;
    if (ty > 220) ty = 220;
    
    // Draw crosshair at tilt position
    draw_line(tx - 10, ty, tx + 10, ty, 0xFFE0);  // Yellow horizontal
    draw_line(tx, ty - 10, tx, ty + 10, 0xFFE0);  // Yellow vertical
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    
    printf("\n=== Intuitive Gravity Cube ===\n");
    printf("Cube tilts to show device orientation\n\n");
    
    // Initialize I2C for IMU
    i2c_init(i2c1, 400000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    
    // Initialize hardware
    printf("Initializing hardware...\n");
    if (DEV_Module_Init() != 0) {
        printf("Hardware init failed!\n");
        return -1;
    }
    
    DEV_SET_PWM(100);
    
    printf("Initializing LCD...\n");
    LCD_1IN28_Init(HORIZONTAL);
    LCD_1IN28_Clear(0x0000);
    
    // Initialize IMU
    printf("Initializing IMU...\n");
    if (!qmi8658_init()) {
        printf("IMU init failed!\n");
        return -1;
    }
    
    printf("Running intuitive cube...\n");
    
    // Low-pass filter for smoothing
    float filtered_ax = 0, filtered_ay = 0, filtered_az = 0;
    const float alpha = 0.1f;  // Filter constant
    
    while (1) {
        // Read accelerometer
        float ax, ay, az;
        read_accel(&ax, &ay, &az);
        
        // Apply low-pass filter
        filtered_ax = alpha * ax + (1.0f - alpha) * filtered_ax;
        filtered_ay = alpha * ay + (1.0f - alpha) * filtered_ay;
        filtered_az = alpha * az + (1.0f - alpha) * filtered_az;
        
        // Calculate tilt angles from gravity
        // When device is flat: ax=0, ay=0, az=1
        // Tilt forward/back: ax changes
        // Tilt left/right: ay changes
        float tilt_x = asinf(filtered_ax);  // Forward/back tilt
        float tilt_y = asinf(filtered_ay);  // Left/right tilt
        
        // Limit angles to reasonable range
        if (tilt_x > PI/4) tilt_x = PI/4;
        if (tilt_x < -PI/4) tilt_x = -PI/4;
        if (tilt_y > PI/4) tilt_y = PI/4;
        if (tilt_y < -PI/4) tilt_y = -PI/4;
        
        // Clear buffer
        clear_buffer(0x0000);
        
        // Copy cube vertices
        vertex_t rotated[8];
        for (int i = 0; i < 8; i++) {
            rotated[i] = cube[i];
        }
        
        // Apply rotations based on tilt
        for (int i = 0; i < 8; i++) {
            rotate_x(&rotated[i], tilt_x);  // Tilt forward/back
            rotate_y(&rotated[i], tilt_y);  // Tilt left/right
        }
        
        // Draw horizon/reference lines
        draw_horizon(filtered_ax, filtered_ay);
        
        // Draw cube
        draw_cube(rotated);
        
        // Draw text indicators
        char buf[50];
        sprintf(buf, "X:%.2f Y:%.2f", filtered_ax, filtered_ay);
        // Note: Would need text rendering to display this
        
        // Display
        display_buffer();
        
        // Debug output
        printf("\rTilt X: %6.2f° Y: %6.2f° | Acc: %.2f %.2f %.2f", 
               tilt_x * 180.0f / PI, tilt_y * 180.0f / PI,
               filtered_ax, filtered_ay, filtered_az);
        
        // ~30 FPS
        sleep_ms(33);
    }
    
    return 0;
}