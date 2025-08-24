/**
 * Rotation Test - Clear visualization of IMU rotation axes
 * Shows each rotation axis separately with clear indicators
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

// Rotation state for visualization
static float roll_angle = 0;
static float pitch_angle = 0;
static float yaw_angle = 0;

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

void draw_rect(int x, int y, int w, int h, uint16_t color) {
    draw_line(x, y, x+w, y, color);
    draw_line(x+w, y, x+w, y+h, color);
    draw_line(x+w, y+h, x, y+h, color);
    draw_line(x, y+h, x, y, color);
}

void fill_rect(int x, int y, int w, int h, uint16_t color) {
    for (int i = y; i < y+h; i++) {
        for (int j = x; j < x+w; j++) {
            set_pixel(j, i, color);
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

// Draw rotation indicator for one axis
void draw_rotation_indicator(int cx, int cy, float angle, uint16_t color, const char* label) {
    (void)label;  // Suppress unused parameter warning
    // Draw circle
    for (int i = 0; i < 360; i += 6) {
        float rad = i * PI / 180.0f;
        int x = cx + (int)(30 * cosf(rad));
        int y = cy + (int)(30 * sinf(rad));
        set_pixel(x, y, 0x4208);  // Dark gray
    }
    
    // Draw arrow showing rotation
    int x = cx + (int)(25 * cosf(angle));
    int y = cy + (int)(25 * sinf(angle));
    draw_line(cx, cy, x, y, color);
    
    // Draw arrowhead
    float arrow_angle1 = angle + 2.5f;
    float arrow_angle2 = angle - 2.5f;
    int ax1 = x + (int)(8 * cosf(arrow_angle1 + PI));
    int ay1 = y + (int)(8 * sinf(arrow_angle1 + PI));
    int ax2 = x + (int)(8 * cosf(arrow_angle2 + PI));
    int ay2 = y + (int)(8 * sinf(arrow_angle2 + PI));
    draw_line(x, y, ax1, ay1, color);
    draw_line(x, y, ax2, ay2, color);
    
    // Draw label (simplified - just colored boxes)
    fill_rect(cx - 5, cy - 40, 10, 3, color);
}

// Draw axis arrows
void draw_3d_axes(int cx, int cy) {
    // X axis (RED) - pointing right
    draw_line(cx, cy, cx + 50, cy, 0xF800);
    draw_line(cx + 50, cy, cx + 45, cy - 5, 0xF800);
    draw_line(cx + 50, cy, cx + 45, cy + 5, 0xF800);
    
    // Y axis (GREEN) - pointing up (screen coords, so negative)
    draw_line(cx, cy, cx, cy - 50, 0x07E0);
    draw_line(cx, cy - 50, cx - 5, cy - 45, 0x07E0);
    draw_line(cx, cy - 50, cx + 5, cy - 45, 0x07E0);
    
    // Z axis (BLUE) - pointing out (diagonal for 3D effect)
    draw_line(cx, cy, cx - 35, cy + 35, 0x001F);
    draw_line(cx - 35, cy + 35, cx - 30, cy + 35, 0x001F);
    draw_line(cx - 35, cy + 35, cx - 35, cy + 30, 0x001F);
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    
    printf("\n=== Rotation Axis Test ===\n");
    printf("Rotate device to test each axis:\n");
    printf("RED (X): Roll - tilt left/right\n");
    printf("GREEN (Y): Pitch - tilt forward/back\n");
    printf("BLUE (Z): Yaw - rotate flat\n\n");
    
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
    
    // Calibrate gyro bias
    printf("Calibrating gyro... hold still\n");
    float gx_bias = 0, gy_bias = 0, gz_bias = 0;
    for (int i = 0; i < 100; i++) {
        int16_t raw_gx = i2c_read_16bit(QMI8658_ADDR, QMI8658_GX_L);
        int16_t raw_gy = i2c_read_16bit(QMI8658_ADDR, QMI8658_GX_L + 2);
        int16_t raw_gz = i2c_read_16bit(QMI8658_ADDR, QMI8658_GX_L + 4);
        gx_bias += raw_gx / 128.0f;
        gy_bias += raw_gy / 128.0f;
        gz_bias += raw_gz / 128.0f;
        sleep_ms(10);
    }
    gx_bias /= 100;
    gy_bias /= 100;
    gz_bias /= 100;
    printf("Calibration done\n");
    
    uint32_t last_time = time_us_32();
    
    while (1) {
        // Read raw gyro values
        int16_t raw_gx = i2c_read_16bit(QMI8658_ADDR, QMI8658_GX_L);
        int16_t raw_gy = i2c_read_16bit(QMI8658_ADDR, QMI8658_GX_L + 2);
        int16_t raw_gz = i2c_read_16bit(QMI8658_ADDR, QMI8658_GX_L + 4);
        
        // Convert to degrees/sec and remove bias
        float gx = (raw_gx / 128.0f) - gx_bias;
        float gy = (raw_gy / 128.0f) - gy_bias;
        float gz = (raw_gz / 128.0f) - gz_bias;
        
        // Calculate dt
        uint32_t now = time_us_32();
        float dt = (now - last_time) / 1000000.0f;
        last_time = now;
        
        // Integrate to get angles (simple integration for visualization)
        roll_angle += gx * dt * PI / 180.0f;
        pitch_angle += gy * dt * PI / 180.0f;
        yaw_angle += gz * dt * PI / 180.0f;
        
        // Wrap angles
        if (roll_angle > PI) roll_angle -= 2*PI;
        if (roll_angle < -PI) roll_angle += 2*PI;
        if (pitch_angle > PI) pitch_angle -= 2*PI;
        if (pitch_angle < -PI) pitch_angle += 2*PI;
        if (yaw_angle > PI) yaw_angle -= 2*PI;
        if (yaw_angle < -PI) yaw_angle += 2*PI;
        
        // Clear and draw
        clear_buffer(0x0000);
        
        // Draw 3D axes reference
        draw_3d_axes(120, 120);
        
        // Draw rotation indicators
        // X-axis rotation (Roll) - RED
        draw_rotation_indicator(60, 60, roll_angle, 0xF800, "X");
        
        // Y-axis rotation (Pitch) - GREEN
        draw_rotation_indicator(180, 60, pitch_angle, 0x07E0, "Y");
        
        // Z-axis rotation (Yaw) - BLUE
        draw_rotation_indicator(120, 180, yaw_angle, 0x001F, "Z");
        
        // Draw rate bars at bottom
        int gx_bar = (int)(fabsf(gx) * 2);
        int gy_bar = (int)(fabsf(gy) * 2);
        int gz_bar = (int)(fabsf(gz) * 2);
        
        if (gx_bar > 40) gx_bar = 40;
        if (gy_bar > 40) gy_bar = 40;
        if (gz_bar > 40) gz_bar = 40;
        
        // X rate (red)
        if (gx > 1) fill_rect(40, 220, gx_bar, 5, 0xF800);
        else if (gx < -1) fill_rect(40 - gx_bar, 220, gx_bar, 5, 0xF800);
        
        // Y rate (green)
        if (gy > 1) fill_rect(100, 220, gy_bar, 5, 0x07E0);
        else if (gy < -1) fill_rect(100 - gy_bar, 220, gy_bar, 5, 0x07E0);
        
        // Z rate (blue)
        if (gz > 1) fill_rect(160, 220, gz_bar, 5, 0x001F);
        else if (gz < -1) fill_rect(160 - gz_bar, 220, gz_bar, 5, 0x001F);
        
        display_buffer();
        
        // Print to console with raw sensor values
        printf("\rRaw: GX:%+4d GY:%+4d GZ:%+4d | °/s: X:%+6.1f Y:%+6.1f Z:%+6.1f", 
               raw_gx, raw_gy, raw_gz, gx, gy, gz);
        
        sleep_ms(20);
    }
    
    return 0;
}