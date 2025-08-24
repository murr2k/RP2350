/**
 * Axis Test - Debug IMU axis orientation
 * Shows raw sensor values to help determine correct axis mapping
 */

#include "pico/stdlib.h"
#include "LCD_1in28.h"
#include "DEV_Config.h"
#include "hardware/i2c.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

#define PI 3.14159265359f
#define WIDTH 240
#define HEIGHT 240

// I2C pins for IMU
#define I2C_SDA_PIN     6
#define I2C_SCL_PIN     7

// QMI8658 registers
#define QMI8658_ADDR    0x6B
#define QMI8658_WHO_AM_I        0x00
#define QMI8658_CTRL1           0x02
#define QMI8658_CTRL2           0x03
#define QMI8658_CTRL3           0x04
#define QMI8658_CTRL7           0x08
#define QMI8658_AX_L            0x35
#define QMI8658_GX_L            0x3B

// Frame buffer
static uint16_t frame_buffer[WIDTH * HEIGHT];

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
    
    i2c_write_register(QMI8658_ADDR, QMI8658_CTRL1, 0x60);
    sleep_ms(10);
    i2c_write_register(QMI8658_ADDR, QMI8658_CTRL2, 0x04);  // 2g
    i2c_write_register(QMI8658_ADDR, QMI8658_CTRL3, 0x54);  // 256 dps
    i2c_write_register(QMI8658_ADDR, QMI8658_CTRL7, 0x03);  // Enable
    
    return true;
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

// Draw a vector from center
void draw_vector(float x, float y, float z, uint16_t color) {
    int cx = 120, cy = 120;
    
    // Scale and project
    int endx = cx + (int)(x * 50);
    int endy = cy - (int)(y * 50);  // Invert Y for screen coords
    
    // Draw main vector
    draw_line(cx, cy, endx, endy, color);
    
    // Draw Z component as thickness (crude 3D effect)
    if (z > 0.1f) {
        draw_line(cx+1, cy, endx+1, endy, color);
        draw_line(cx-1, cy, endx-1, endy, color);
    }
    if (z > 0.5f) {
        draw_line(cx, cy+1, endx, endy+1, color);
        draw_line(cx, cy-1, endx, endy-1, color);
    }
}

// Draw axis labels
void draw_labels(void) {
    // Draw coordinate axes
    draw_line(20, 120, 220, 120, 0x0410);  // X axis
    draw_line(120, 20, 120, 220, 0x0410);  // Y axis
    
    // Draw labels (simplified as markers)
    // +X
    draw_line(210, 115, 215, 120, 0x0410);
    draw_line(210, 125, 215, 120, 0x0410);
    
    // -Y (up on screen)
    draw_line(115, 30, 120, 25, 0x0410);
    draw_line(125, 30, 120, 25, 0x0410);
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    
    printf("\n=== IMU Axis Test ===\n");
    printf("Tilt device to see axis responses\n");
    printf("Hold flat: Z=1g (gravity down)\n");
    printf("Tilt right: +X\n");
    printf("Tilt forward: +Y\n\n");
    
    // Initialize I2C
    i2c_init(i2c1, 400000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    
    // Initialize hardware
    if (DEV_Module_Init() != 0) {
        return -1;
    }
    
    DEV_SET_PWM(100);
    LCD_1IN28_Init(HORIZONTAL);
    LCD_1IN28_Clear(0x0000);
    
    // Initialize IMU
    if (!qmi8658_init()) {
        printf("IMU init failed!\n");
        return -1;
    }
    
    while (1) {
        // Read raw values
        int16_t raw_ax = i2c_read_16bit(QMI8658_ADDR, QMI8658_AX_L);
        int16_t raw_ay = i2c_read_16bit(QMI8658_ADDR, QMI8658_AX_L + 2);
        int16_t raw_az = i2c_read_16bit(QMI8658_ADDR, QMI8658_AX_L + 4);
        
        int16_t raw_gx = i2c_read_16bit(QMI8658_ADDR, QMI8658_GX_L);
        int16_t raw_gy = i2c_read_16bit(QMI8658_ADDR, QMI8658_GX_L + 2);
        int16_t raw_gz = i2c_read_16bit(QMI8658_ADDR, QMI8658_GX_L + 4);
        
        // Convert to g's
        float ax = raw_ax / 16384.0f;
        float ay = raw_ay / 16384.0f;
        float az = raw_az / 16384.0f;
        
        float gx = raw_gx / 128.0f;  // degrees/sec
        float gy = raw_gy / 128.0f;
        float gz = raw_gz / 128.0f;
        
        // Clear and draw
        clear_buffer(0x0000);
        draw_labels();
        
        // Draw accelerometer vector (RED)
        draw_vector(ax, ay, az, 0xF800);
        
        // Draw gyro vector (GREEN) - scaled down
        draw_vector(gx/100.0f, gy/100.0f, gz/100.0f, 0x07E0);
        
        // Draw text indicators showing raw values
        // Corner indicators for magnitude
        int ax_bar = (int)(fabsf(ax) * 50);
        int ay_bar = (int)(fabsf(ay) * 50);
        int az_bar = (int)(fabsf(az) * 50);
        
        // Top left: X accel
        if (ax > 0.1f) draw_line(10, 10, 10 + ax_bar, 10, 0xF800);
        else if (ax < -0.1f) draw_line(10, 15, 10 + ax_bar, 15, 0xF880);
        
        // Top right: Y accel  
        if (ay > 0.1f) draw_line(230, 10, 230 - ay_bar, 10, 0x07E0);
        else if (ay < -0.1f) draw_line(230, 15, 230 - ay_bar, 15, 0x07F0);
        
        // Bottom: Z accel
        if (az > 0.1f) draw_line(120 - az_bar/2, 230, 120 + az_bar/2, 230, 0x001F);
        else if (az < -0.1f) draw_line(120 - az_bar/2, 225, 120 + az_bar/2, 225, 0x001F);
        
        display_buffer();
        
        // Print to console
        printf("\rAccel: X:%+.2f Y:%+.2f Z:%+.2f | Gyro: X:%+6.1f Y:%+6.1f Z:%+6.1f", 
               ax, ay, az, gx, gy, gz);
        
        sleep_ms(50);
    }
    
    return 0;
}