/**
 * Gravity-Locked Rubik's Cube Demo
 * The cube stays level to Earth while the device moves
 * Uses QMI8658 IMU for gravity detection and complementary filtering
 */

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// LCD Pin definitions
#define LCD_DC_PIN      8
#define LCD_CS_PIN      9
#define LCD_CLK_PIN     10
#define LCD_MOSI_PIN    11
#define LCD_RST_PIN     12
#define LCD_BL_PIN      25

// I2C pins for IMU
#define I2C_SDA_PIN     6
#define I2C_SCL_PIN     7

// QMI8658 I2C address
#define QMI8658_ADDR    0x6B

// QMI8658 Registers
#define QMI8658_WHO_AM_I        0x00
#define QMI8658_CTRL1           0x02
#define QMI8658_CTRL2           0x03
#define QMI8658_CTRL3           0x04
#define QMI8658_CTRL7           0x08
#define QMI8658_AX_L            0x35
#define QMI8658_GX_L            0x3B

// Display dimensions
#define LCD_WIDTH  240
#define LCD_HEIGHT 240
#define CENTER_X   120
#define CENTER_Y   120

// Math constants
#define PI 3.14159265359f
#define DEG_TO_RAD (PI / 180.0f)
#define RAD_TO_DEG (180.0f / PI)

// Complementary filter constant (0.98 = trust gyro more, 0.90 = trust accel more)
#define ALPHA 0.95f

// Colors
#define BLACK   0x0000
#define WHITE   0xFFFF
#define RED     0xF800
#define GREEN   0x07E0
#define BLUE    0x001F
#define YELLOW  0xFFE0
#define ORANGE  0xFD20
#define CYAN    0x07FF
#define MAGENTA 0xF81F

// 3D cube vertices (centered at origin, size 60)
static const float cube_vertices[8][3] = {
    {-30, -30, -30}, {30, -30, -30}, {30, 30, -30}, {-30, 30, -30},  // Back face
    {-30, -30,  30}, {30, -30,  30}, {30, 30,  30}, {-30, 30,  30}   // Front face
};

// Cube edges (vertex pairs)
static const uint8_t cube_edges[12][2] = {
    {0,1}, {1,2}, {2,3}, {3,0},  // Back face
    {4,5}, {5,6}, {6,7}, {7,4},  // Front face
    {0,4}, {1,5}, {2,6}, {3,7}   // Connecting edges
};

// Rubik's cube face colors (simplified - just colored squares)
typedef struct {
    uint8_t vertices[4];  // Indices of the 4 vertices
    uint16_t color;       // Face color
} cube_face_t;

static const cube_face_t cube_faces[6] = {
    {{0,1,2,3}, RED},     // Back
    {{4,5,6,7}, ORANGE},  // Front
    {{0,3,7,4}, GREEN},   // Left
    {{1,2,6,5}, BLUE},    // Right
    {{0,1,5,4}, WHITE},   // Bottom
    {{2,3,7,6}, YELLOW}   // Top
};

// IMU data structure
typedef struct {
    float ax, ay, az;  // Accelerometer (g)
    float gx, gy, gz;  // Gyroscope (rad/s)
} imu_data_t;

// Global state
static float phi = 0.0f;      // Current rotation angle (radians)
static float phi_cal = 0.0f;  // Calibration offset
static uint32_t last_time = 0;

// LCD functions
static void lcd_write_command(uint8_t cmd) {
    gpio_put(LCD_DC_PIN, 0);
    spi_write_blocking(spi1, &cmd, 1);
}

static void lcd_write_data(uint8_t data) {
    gpio_put(LCD_DC_PIN, 1);
    spi_write_blocking(spi1, &data, 1);
}

static void lcd_write_data16(uint16_t data) {
    uint8_t buf[2];
    buf[0] = data >> 8;
    buf[1] = data & 0xFF;
    gpio_put(LCD_DC_PIN, 1);
    spi_write_blocking(spi1, buf, 2);
}

static void lcd_reset(void) {
    gpio_put(LCD_RST_PIN, 1);
    sleep_ms(100);
    gpio_put(LCD_RST_PIN, 0);
    sleep_ms(100);
    gpio_put(LCD_RST_PIN, 1);
    gpio_put(LCD_CS_PIN, 0);  // Keep CS LOW
    sleep_ms(100);
}

static void lcd_init_reg(void) {
    // Waveshare initialization sequence (abbreviated for space)
    lcd_write_command(0xEF);
    lcd_write_command(0xEB);
    lcd_write_data(0x14);
    
    lcd_write_command(0xFE);
    lcd_write_command(0xEF);
    
    lcd_write_command(0xEB);
    lcd_write_data(0x14);
    
    lcd_write_command(0x84);
    lcd_write_data(0x40);
    
    lcd_write_command(0x85);
    lcd_write_data(0xFF);
    
    lcd_write_command(0x86);
    lcd_write_data(0xFF);
    
    lcd_write_command(0x87);
    lcd_write_data(0xFF);
    
    lcd_write_command(0x88);
    lcd_write_data(0x0A);
    
    lcd_write_command(0x89);
    lcd_write_data(0x21);
    
    lcd_write_command(0x8A);
    lcd_write_data(0x00);
    
    lcd_write_command(0x8B);
    lcd_write_data(0x80);
    
    lcd_write_command(0x8C);
    lcd_write_data(0x01);
    
    lcd_write_command(0x8D);
    lcd_write_data(0x01);
    
    lcd_write_command(0x8E);
    lcd_write_data(0xFF);
    
    lcd_write_command(0x8F);
    lcd_write_data(0xFF);
    
    lcd_write_command(0xB6);
    lcd_write_data(0x00);
    lcd_write_data(0x20);
    
    lcd_write_command(0x36);
    lcd_write_data(0x08);
    
    lcd_write_command(0x3A);
    lcd_write_data(0x05);
    
    // Extended init commands...
    lcd_write_command(0x90);
    lcd_write_data(0x08);
    lcd_write_data(0x08);
    lcd_write_data(0x08);
    lcd_write_data(0x08);
    
    lcd_write_command(0xBD);
    lcd_write_data(0x06);
    
    lcd_write_command(0xBC);
    lcd_write_data(0x00);
    
    lcd_write_command(0xFF);
    lcd_write_data(0x60);
    lcd_write_data(0x01);
    lcd_write_data(0x04);
    
    lcd_write_command(0xC3);
    lcd_write_data(0x13);
    lcd_write_command(0xC4);
    lcd_write_data(0x13);
    
    lcd_write_command(0xC9);
    lcd_write_data(0x22);
    
    lcd_write_command(0xBE);
    lcd_write_data(0x11);
    
    lcd_write_command(0xE1);
    lcd_write_data(0x10);
    lcd_write_data(0x0E);
    
    lcd_write_command(0xDF);
    lcd_write_data(0x21);
    lcd_write_data(0x0C);
    lcd_write_data(0x02);
    
    // Gamma settings
    lcd_write_command(0xF0);
    lcd_write_data(0x45);
    lcd_write_data(0x09);
    lcd_write_data(0x08);
    lcd_write_data(0x08);
    lcd_write_data(0x26);
    lcd_write_data(0x2A);
    
    lcd_write_command(0xF1);
    lcd_write_data(0x43);
    lcd_write_data(0x70);
    lcd_write_data(0x72);
    lcd_write_data(0x36);
    lcd_write_data(0x37);
    lcd_write_data(0x6F);
    
    // More init...
    lcd_write_command(0x35);
    lcd_write_command(0x21);
    
    lcd_write_command(0x11);
    sleep_ms(120);
    
    lcd_write_command(0x29);
    sleep_ms(20);
}

static void lcd_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    lcd_write_command(0x2A);
    lcd_write_data(x0 >> 8);
    lcd_write_data(x0 & 0xFF);
    lcd_write_data(x1 >> 8);
    lcd_write_data(x1 & 0xFF);
    
    lcd_write_command(0x2B);
    lcd_write_data(y0 >> 8);
    lcd_write_data(y0 & 0xFF);
    lcd_write_data(y1 >> 8);
    lcd_write_data(y1 & 0xFF);
    
    lcd_write_command(0x2C);
}

static void lcd_clear(uint16_t color) {
    lcd_set_window(0, 0, 239, 239);
    gpio_put(LCD_DC_PIN, 1);
    
    for (int i = 0; i < 240 * 240; i++) {
        lcd_write_data16(color);
    }
}

static void lcd_draw_pixel(int16_t x, int16_t y, uint16_t color) {
    if (x < 0 || x >= LCD_WIDTH || y < 0 || y >= LCD_HEIGHT) return;
    
    lcd_set_window(x, y, x, y);
    lcd_write_data16(color);
}

// Bresenham's line algorithm
static void lcd_draw_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) {
    int16_t dx = abs(x1 - x0);
    int16_t dy = abs(y1 - y0);
    int16_t sx = (x0 < x1) ? 1 : -1;
    int16_t sy = (y0 < y1) ? 1 : -1;
    int16_t err = dx - dy;
    
    while (1) {
        lcd_draw_pixel(x0, y0, color);
        
        if (x0 == x1 && y0 == y1) break;
        
        int16_t e2 = 2 * err;
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

// Draw filled triangle using scanline algorithm
static void lcd_draw_filled_triangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, 
                                     int16_t x2, int16_t y2, uint16_t color) {
    // Sort vertices by y coordinate
    if (y0 > y1) { int16_t t = y0; y0 = y1; y1 = t; t = x0; x0 = x1; x1 = t; }
    if (y1 > y2) { int16_t t = y1; y1 = y2; y2 = t; t = x1; x1 = x2; x2 = t; }
    if (y0 > y1) { int16_t t = y0; y0 = y1; y1 = t; t = x0; x0 = x1; x1 = t; }
    
    // Draw scanlines
    for (int16_t y = y0; y <= y2; y++) {
        int16_t xa, xb;
        
        if (y <= y1) {
            xa = x0 + (x1 - x0) * (y - y0) / (y1 - y0 + 1);
            xb = x0 + (x2 - x0) * (y - y0) / (y2 - y0 + 1);
        } else {
            xa = x1 + (x2 - x1) * (y - y1) / (y2 - y1 + 1);
            xb = x0 + (x2 - x0) * (y - y0) / (y2 - y0 + 1);
        }
        
        if (xa > xb) { int16_t t = xa; xa = xb; xb = t; }
        
        for (int16_t x = xa; x <= xb; x++) {
            lcd_draw_pixel(x, y, color);
        }
    }
}

// I2C functions for IMU
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
        printf("QMI8658 not found!\n");
        return false;
    }
    
    // Reset
    i2c_write_register(QMI8658_ADDR, QMI8658_CTRL1, 0x60);
    sleep_ms(10);
    
    // Configure accelerometer: 2g range, 500Hz ODR
    i2c_write_register(QMI8658_ADDR, QMI8658_CTRL2, 0x05);
    
    // Configure gyroscope: 512 dps range, 500Hz ODR  
    i2c_write_register(QMI8658_ADDR, QMI8658_CTRL3, 0x55);
    
    // Enable sensors
    i2c_write_register(QMI8658_ADDR, QMI8658_CTRL7, 0x03);
    
    return true;
}

// Read IMU data
static void qmi8658_read(imu_data_t *data) {
    // Read raw values
    int16_t ax = i2c_read_16bit(QMI8658_ADDR, QMI8658_AX_L);
    int16_t ay = i2c_read_16bit(QMI8658_ADDR, QMI8658_AX_L + 2);
    int16_t az = i2c_read_16bit(QMI8658_ADDR, QMI8658_AX_L + 4);
    
    int16_t gx = i2c_read_16bit(QMI8658_ADDR, QMI8658_GX_L);
    int16_t gy = i2c_read_16bit(QMI8658_ADDR, QMI8658_GX_L + 2);
    int16_t gz = i2c_read_16bit(QMI8658_ADDR, QMI8658_GX_L + 4);
    
    // Convert to physical units
    // Accelerometer: 2g range, sensitivity = 16384 LSB/g
    data->ax = ax / 16384.0f;
    data->ay = ay / 16384.0f;
    data->az = az / 16384.0f;
    
    // Gyroscope: 512 dps range, sensitivity = 64 LSB/dps
    // Convert to rad/s
    data->gx = (gx / 64.0f) * DEG_TO_RAD;
    data->gy = (gy / 64.0f) * DEG_TO_RAD;
    data->gz = (gz / 64.0f) * DEG_TO_RAD;
}

// Normalize a 3D vector
static void normalize3(float *x, float *y, float *z) {
    float mag = sqrtf((*x) * (*x) + (*y) * (*y) + (*z) * (*z));
    if (mag > 0.0f) {
        *x /= mag;
        *y /= mag;
        *z /= mag;
    }
}

// Calibrate IMU (find initial gravity angle)
static float calibrate_imu(void) {
    printf("Calibrating IMU...\n");
    
    float ax_sum = 0, ay_sum = 0, az_sum = 0;
    const int samples = 50;
    
    for (int i = 0; i < samples; i++) {
        imu_data_t data;
        qmi8658_read(&data);
        ax_sum += data.ax;
        ay_sum += data.ay;
        az_sum += data.az;
        sleep_ms(10);
    }
    
    float ax = ax_sum / samples;
    float ay = ay_sum / samples;
    float az = az_sum / samples;
    
    normalize3(&ax, &ay, &az);
    
    // Calculate initial angle of gravity in screen plane
    float phi0 = atan2f(ax, ay);
    
    printf("Calibration done. Initial angle: %.2f deg\n", phi0 * RAD_TO_DEG);
    return phi0;
}

// 3D rotation matrix (rotate around Z axis)
static void rotate_vertex(float x, float y, float z, float angle, float *rx, float *ry, float *rz) {
    float cos_a = cosf(angle);
    float sin_a = sinf(angle);
    
    *rx = x * cos_a - y * sin_a;
    *ry = x * sin_a + y * cos_a;
    *rz = z;
}

// Project 3D point to 2D screen
static void project_3d_to_2d(float x, float y, float z, int16_t *sx, int16_t *sy) {
    // Simple orthographic projection with perspective hint
    float scale = 1.0f + z / 200.0f;  // Slight perspective effect
    *sx = (int16_t)(CENTER_X + x * scale);
    *sy = (int16_t)(CENTER_Y + y * scale);
}

// Draw the 3D cube
static void draw_cube(float rotation) {
    // Transform and project all vertices
    int16_t screen_vertices[8][2];
    float rotated_vertices[8][3];
    
    for (int i = 0; i < 8; i++) {
        rotate_vertex(cube_vertices[i][0], cube_vertices[i][1], cube_vertices[i][2],
                     rotation, &rotated_vertices[i][0], &rotated_vertices[i][1], 
                     &rotated_vertices[i][2]);
        
        project_3d_to_2d(rotated_vertices[i][0], rotated_vertices[i][1], 
                        rotated_vertices[i][2], &screen_vertices[i][0], 
                        &screen_vertices[i][1]);
    }
    
    // Calculate face depths for sorting (painter's algorithm)
    float face_depths[6];
    for (int i = 0; i < 6; i++) {
        // Average Z of face vertices
        float z_sum = 0;
        for (int j = 0; j < 4; j++) {
            z_sum += rotated_vertices[cube_faces[i].vertices[j]][2];
        }
        face_depths[i] = z_sum / 4.0f;
    }
    
    // Sort faces by depth (back to front)
    int face_order[6] = {0, 1, 2, 3, 4, 5};
    for (int i = 0; i < 5; i++) {
        for (int j = i + 1; j < 6; j++) {
            if (face_depths[face_order[i]] > face_depths[face_order[j]]) {
                int temp = face_order[i];
                face_order[i] = face_order[j];
                face_order[j] = temp;
            }
        }
    }
    
    // Draw faces from back to front
    for (int i = 0; i < 6; i++) {
        int face_idx = face_order[i];
        const cube_face_t *face = &cube_faces[face_idx];
        
        // Check if face is visible (normal pointing towards viewer)
        int16_t x0 = screen_vertices[face->vertices[0]][0];
        int16_t y0 = screen_vertices[face->vertices[0]][1];
        int16_t x1 = screen_vertices[face->vertices[1]][0];
        int16_t y1 = screen_vertices[face->vertices[1]][1];
        int16_t x2 = screen_vertices[face->vertices[2]][0];
        int16_t y2 = screen_vertices[face->vertices[2]][1];
        
        // Cross product for backface culling
        int32_t cross = (x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0);
        
        if (cross > 0) {  // Face is visible
            // Draw two triangles to fill the quad
            lcd_draw_filled_triangle(
                screen_vertices[face->vertices[0]][0],
                screen_vertices[face->vertices[0]][1],
                screen_vertices[face->vertices[1]][0],
                screen_vertices[face->vertices[1]][1],
                screen_vertices[face->vertices[2]][0],
                screen_vertices[face->vertices[2]][1],
                face->color
            );
            
            lcd_draw_filled_triangle(
                screen_vertices[face->vertices[0]][0],
                screen_vertices[face->vertices[0]][1],
                screen_vertices[face->vertices[2]][0],
                screen_vertices[face->vertices[2]][1],
                screen_vertices[face->vertices[3]][0],
                screen_vertices[face->vertices[3]][1],
                face->color
            );
        }
    }
    
    // Draw edges for better definition
    for (int i = 0; i < 12; i++) {
        lcd_draw_line(
            screen_vertices[cube_edges[i][0]][0],
            screen_vertices[cube_edges[i][0]][1],
            screen_vertices[cube_edges[i][1]][0],
            screen_vertices[cube_edges[i][1]][1],
            BLACK
        );
    }
}

// Draw a reference circle
static void draw_reference_circle(void) {
    const int radius = 115;
    for (int angle = 0; angle < 360; angle += 3) {
        float rad = angle * DEG_TO_RAD;
        int16_t x = CENTER_X + radius * cosf(rad);
        int16_t y = CENTER_Y + radius * sinf(rad);
        lcd_draw_pixel(x, y, CYAN);
        lcd_draw_pixel(x+1, y, CYAN);
        lcd_draw_pixel(x, y+1, CYAN);
    }
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    
    printf("\n=== Gravity-Locked Rubik's Cube Demo ===\n");
    printf("The cube stays level while you move the device!\n\n");
    
    // Initialize I2C for IMU
    i2c_init(i2c1, 400000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    
    // Initialize LCD
    gpio_init(LCD_DC_PIN);
    gpio_set_dir(LCD_DC_PIN, GPIO_OUT);
    
    gpio_init(LCD_CS_PIN);
    gpio_set_dir(LCD_CS_PIN, GPIO_OUT);
    gpio_put(LCD_CS_PIN, 1);
    
    gpio_init(LCD_RST_PIN);
    gpio_set_dir(LCD_RST_PIN, GPIO_OUT);
    
    // Initialize SPI
    spi_init(spi1, 62500000);
    gpio_set_function(LCD_CLK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(LCD_MOSI_PIN, GPIO_FUNC_SPI);
    
    // Initialize backlight
    gpio_set_function(LCD_BL_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(LCD_BL_PIN);
    pwm_set_wrap(slice, 255);
    pwm_set_gpio_level(LCD_BL_PIN, 200);
    pwm_set_enabled(slice, true);
    
    // Initialize LCD
    printf("Initializing LCD...\n");
    lcd_reset();
    lcd_init_reg();
    lcd_clear(BLACK);
    
    // Initialize IMU
    printf("Initializing IMU...\n");
    if (!qmi8658_init()) {
        printf("Failed to initialize IMU!\n");
        while (1) {
            lcd_clear(RED);
            sleep_ms(500);
            lcd_clear(BLACK);
            sleep_ms(500);
        }
    }
    
    // Calibrate IMU
    phi_cal = calibrate_imu();
    
    // Draw initial reference circle
    draw_reference_circle();
    
    printf("Starting gravity-locked cube...\n");
    
    last_time = time_us_32();
    
    while (1) {
        // Read IMU data
        imu_data_t imu;
        qmi8658_read(&imu);
        
        // Calculate time delta
        uint32_t now = time_us_32();
        float dt = (now - last_time) / 1000000.0f;
        last_time = now;
        
        // Normalize gravity vector
        float ax = imu.ax, ay = imu.ay, az = imu.az;
        normalize3(&ax, &ay, &az);
        
        // Project gyro onto gravity axis (rotation about "down")
        float w_parallel = ax * imu.gx + ay * imu.gy + az * imu.gz;
        
        // Gyro integration
        float phi_gyro = phi + w_parallel * dt;
        
        // Accelerometer-only angle
        float phi_accel = atan2f(ax, ay) - phi_cal;
        
        // Wrap to [-PI, PI]
        while (phi_accel > PI) phi_accel -= 2 * PI;
        while (phi_accel < -PI) phi_accel += 2 * PI;
        
        // Complementary filter
        phi = ALPHA * phi_gyro + (1.0f - ALPHA) * phi_accel;
        
        // Clear screen
        lcd_clear(BLACK);
        
        // Redraw reference circle
        draw_reference_circle();
        
        // Draw cube with opposite rotation to stay "level"
        draw_cube(-phi);
        
        // Debug output
        printf("\rAngle: %.1f° | Acc: %.2f %.2f %.2f | Gyro: %.1f %.1f %.1f",
               phi * RAD_TO_DEG, imu.ax, imu.ay, imu.az,
               imu.gx * RAD_TO_DEG, imu.gy * RAD_TO_DEG, imu.gz * RAD_TO_DEG);
        
        // ~60 FPS
        sleep_ms(16);
    }
    
    return 0;
}