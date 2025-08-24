/**
 * Wireframe Gravity-Locked Cube Demo
 * Simple wireframe cube that stays level to Earth
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

// Complementary filter constant
#define ALPHA 0.98f

// Colors
#define BLACK   0x0000
#define WHITE   0xFFFF
#define GREEN   0x07E0
#define CYAN    0x07FF
#define RED     0xF800
#define YELLOW  0xFFE0

// Frame buffer for smoother rendering
static uint16_t frame_buffer[240 * 240];

// 3D cube vertices (8 corners of a cube)
static const float cube_vertices[8][3] = {
    {-50, -50, -50}, {50, -50, -50}, {50, 50, -50}, {-50, 50, -50},  // Back face
    {-50, -50,  50}, {50, -50,  50}, {50, 50,  50}, {-50, 50,  50}   // Front face
};

// Cube edges (12 edges connecting vertices)
static const uint8_t cube_edges[12][2] = {
    {0,1}, {1,2}, {2,3}, {3,0},  // Back face
    {4,5}, {5,6}, {6,7}, {7,4},  // Front face
    {0,4}, {1,5}, {2,6}, {3,7}   // Connecting edges
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
    // Essential initialization only
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
    
    lcd_write_command(0x21);  // Inversion ON
    
    lcd_write_command(0x11);  // Sleep out
    sleep_ms(120);
    
    lcd_write_command(0x29);  // Display ON
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

static void lcd_push_frame(void) {
    lcd_set_window(0, 0, 239, 239);
    gpio_put(LCD_DC_PIN, 1);
    
    // Send frame buffer in one go
    spi_write_blocking(spi1, (uint8_t*)frame_buffer, sizeof(frame_buffer));
}

// Clear frame buffer
static void clear_frame(uint16_t color) {
    for (int i = 0; i < 240 * 240; i++) {
        frame_buffer[i] = (color >> 8) | ((color & 0xFF) << 8);  // Swap bytes for LCD
    }
}

// Set pixel in frame buffer
static void set_pixel(int16_t x, int16_t y, uint16_t color) {
    if (x >= 0 && x < LCD_WIDTH && y >= 0 && y < LCD_HEIGHT) {
        frame_buffer[y * LCD_WIDTH + x] = (color >> 8) | ((color & 0xFF) << 8);
    }
}

// Bresenham's line algorithm
static void draw_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) {
    int16_t dx = abs(x1 - x0);
    int16_t dy = abs(y1 - y0);
    int16_t sx = (x0 < x1) ? 1 : -1;
    int16_t sy = (y0 < y1) ? 1 : -1;
    int16_t err = dx - dy;
    
    while (1) {
        set_pixel(x0, y0, color);
        
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

// Draw circle
static void draw_circle(int16_t cx, int16_t cy, int16_t r, uint16_t color) {
    for (int angle = 0; angle < 360; angle += 2) {
        float rad = angle * DEG_TO_RAD;
        int16_t x = cx + r * cosf(rad);
        int16_t y = cy + r * sinf(rad);
        set_pixel(x, y, color);
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
        return false;
    }
    
    // Reset
    i2c_write_register(QMI8658_ADDR, QMI8658_CTRL1, 0x60);
    sleep_ms(10);
    
    // Configure accelerometer: 2g range, 250Hz ODR
    i2c_write_register(QMI8658_ADDR, QMI8658_CTRL2, 0x04);
    
    // Configure gyroscope: 256 dps range, 250Hz ODR  
    i2c_write_register(QMI8658_ADDR, QMI8658_CTRL3, 0x54);
    
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
    data->ax = ax / 16384.0f;  // 2g range
    data->ay = ay / 16384.0f;
    data->az = az / 16384.0f;
    
    data->gx = (gx / 128.0f) * DEG_TO_RAD;  // 256 dps range
    data->gy = (gy / 128.0f) * DEG_TO_RAD;
    data->gz = (gz / 128.0f) * DEG_TO_RAD;
}

// 3D rotation around Z axis
static void rotate_vertex(float x, float y, float z, float angle, float *rx, float *ry, float *rz) {
    float cos_a = cosf(angle);
    float sin_a = sinf(angle);
    
    *rx = x * cos_a - y * sin_a;
    *ry = x * sin_a + y * cos_a;
    *rz = z;
}

// Project 3D to 2D
static void project_3d_to_2d(float x, float y, float z, int16_t *sx, int16_t *sy) {
    // Simple perspective projection
    float perspective = 1.0f / (1.0f - z / 300.0f);
    *sx = (int16_t)(CENTER_X + x * perspective);
    *sy = (int16_t)(CENTER_Y + y * perspective);
}

// Draw wireframe cube
static void draw_wireframe_cube(float rotation) {
    // Transform and project vertices
    int16_t screen_vertices[8][2];
    
    for (int i = 0; i < 8; i++) {
        float rx, ry, rz;
        rotate_vertex(cube_vertices[i][0], cube_vertices[i][1], cube_vertices[i][2],
                     rotation, &rx, &ry, &rz);
        
        project_3d_to_2d(rx, ry, rz, &screen_vertices[i][0], &screen_vertices[i][1]);
    }
    
    // Draw all edges
    for (int i = 0; i < 12; i++) {
        uint16_t color = GREEN;
        
        // Color code different edge groups
        if (i < 4) color = CYAN;        // Back face
        else if (i < 8) color = YELLOW; // Front face
        else color = WHITE;             // Connecting edges
        
        draw_line(screen_vertices[cube_edges[i][0]][0],
                 screen_vertices[cube_edges[i][0]][1],
                 screen_vertices[cube_edges[i][1]][0],
                 screen_vertices[cube_edges[i][1]][1],
                 color);
    }
}

// Calibrate IMU
static float calibrate_imu(void) {
    printf("Calibrating... Hold still\n");
    
    float ax_sum = 0, ay_sum = 0;
    const int samples = 30;
    
    for (int i = 0; i < samples; i++) {
        imu_data_t data;
        qmi8658_read(&data);
        ax_sum += data.ax;
        ay_sum += data.ay;
        sleep_ms(20);
    }
    
    float ax = ax_sum / samples;
    float ay = ay_sum / samples;
    
    float phi0 = atan2f(ax, ay);
    printf("Calibration done: %.2f°\n", phi0 * RAD_TO_DEG);
    return phi0;
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    
    printf("\n=== Wireframe Gravity-Locked Cube ===\n");
    
    // Initialize I2C for IMU
    i2c_init(i2c1, 400000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    
    // Initialize LCD pins
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
    printf("Init LCD...\n");
    lcd_reset();
    lcd_init_reg();
    
    // Clear screen
    clear_frame(BLACK);
    lcd_push_frame();
    
    // Initialize IMU
    printf("Init IMU...\n");
    if (!qmi8658_init()) {
        printf("IMU init failed!\n");
        // Show error pattern
        while (1) {
            clear_frame(RED);
            lcd_push_frame();
            sleep_ms(500);
            clear_frame(BLACK);
            lcd_push_frame();
            sleep_ms(500);
        }
    }
    
    // Calibrate
    phi_cal = calibrate_imu();
    
    printf("Running...\n");
    last_time = time_us_32();
    
    while (1) {
        // Read IMU
        imu_data_t imu;
        qmi8658_read(&imu);
        
        // Time delta
        uint32_t now = time_us_32();
        float dt = (now - last_time) / 1000000.0f;
        last_time = now;
        
        // Normalize gravity
        float mag = sqrtf(imu.ax * imu.ax + imu.ay * imu.ay + imu.az * imu.az);
        if (mag > 0) {
            imu.ax /= mag;
            imu.ay /= mag;
            imu.az /= mag;
        }
        
        // Gyro component parallel to gravity
        float w_par = imu.ax * imu.gx + imu.ay * imu.gy + imu.az * imu.gz;
        
        // Integrate gyro
        float phi_gyro = phi + w_par * dt;
        
        // Accel angle
        float phi_accel = atan2f(imu.ax, imu.ay) - phi_cal;
        
        // Wrap angle
        while (phi_accel > PI) phi_accel -= 2 * PI;
        while (phi_accel < -PI) phi_accel += 2 * PI;
        
        // Complementary filter
        phi = ALPHA * phi_gyro + (1.0f - ALPHA) * phi_accel;
        
        // Clear and draw
        clear_frame(BLACK);
        
        // Draw reference circle
        draw_circle(CENTER_X, CENTER_Y, 115, CYAN);
        
        // Draw wireframe cube (rotate opposite to stay level)
        draw_wireframe_cube(-phi);
        
        // Push frame to LCD
        lcd_push_frame();
        
        // Status
        printf("\rAngle: %.1f°", phi * RAD_TO_DEG);
        
        // ~30 FPS
        sleep_ms(33);
    }
    
    return 0;
}