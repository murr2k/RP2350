/**
 * Gravity-Locked Cube - FINAL VERSION
 * Cube stays level to Earth while device moves
 * Uses buffered rendering + QMI8658 IMU
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
#define QMI8658_GX_L            0x3B

// Complementary filter constant (0.98 = trust gyro more)
#define ALPHA 0.98f

// Frame buffer
static uint16_t frame_buffer[WIDTH * HEIGHT];

// Cube vertices
typedef struct {
    float x, y, z;
} vertex_t;

vertex_t cube[8] = {
    {-40, -40, -40}, {40, -40, -40}, {40, 40, -40}, {-40, 40, -40},
    {-40, -40,  40}, {40, -40,  40}, {40, 40,  40}, {-40, 40,  40}
};

const int edges[12][2] = {
    {0,1}, {1,2}, {2,3}, {3,0},
    {4,5}, {5,6}, {6,7}, {7,4},
    {0,4}, {1,5}, {2,6}, {3,7}
};

// IMU data
typedef struct {
    float ax, ay, az;  // Accelerometer (g)
    float gx, gy, gz;  // Gyroscope (rad/s)
} imu_data_t;

// Global rotation state
static float phi = 0.0f;      // Current angle
static float phi_cal = 0.0f;  // Calibration offset
static uint32_t last_time = 0;

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

// Read IMU data
static void qmi8658_read(imu_data_t *data) {
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
    
    // Convert to rad/s
    data->gx = (gx / 128.0f) * (PI / 180.0f);  // 256 dps
    data->gy = (gy / 128.0f) * (PI / 180.0f);
    data->gz = (gz / 128.0f) * (PI / 180.0f);
}

// Calibrate IMU (find initial gravity angle)
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
    printf("Calibration done: %.2f°\n", phi0 * 180.0f / PI);
    return phi0;
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

// 3D rotation (around Z axis for gravity lock)
void rotate_z(vertex_t *v, float angle) {
    float cos_a = cosf(angle);
    float sin_a = sinf(angle);
    float x = v->x;
    float y = v->y;
    v->x = x * cos_a - y * sin_a;
    v->y = x * sin_a + y * cos_a;
}

// Project 3D to 2D
void project(vertex_t v, int *x, int *y) {
    float distance = 150.0f;
    float z = v.z + distance;
    *x = (int)(120 + (v.x * 100) / z);
    *y = (int)(120 + (v.y * 100) / z);
}

// Draw wireframe cube
void draw_cube(vertex_t *vertices) {
    int screen[8][2];
    
    for (int i = 0; i < 8; i++) {
        project(vertices[i], &screen[i][0], &screen[i][1]);
    }
    
    for (int i = 0; i < 12; i++) {
        int v0 = edges[i][0];
        int v1 = edges[i][1];
        
        uint16_t color = 0xFFFF;
        if (i < 4) color = 0xF800;      // Red - back
        else if (i < 8) color = 0x07E0; // Green - front
        else color = 0x001F;             // Blue - connecting
        
        draw_line(screen[v0][0], screen[v0][1], 
                 screen[v1][0], screen[v1][1], color);
    }
}

// Draw circle
void draw_circle(int cx, int cy, int r, uint16_t color) {
    for (int angle = 0; angle < 360; angle += 3) {
        int x = cx + (int)(r * cosf(angle * PI / 180));
        int y = cy + (int)(r * sinf(angle * PI / 180));
        set_pixel(x, y, color);
    }
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    
    printf("\n=== Gravity-Locked Cube ===\n");
    printf("The cube stays level while you move!\n\n");
    
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
        printf("IMU init failed! Running without gravity lock.\n");
        // Continue anyway for demo
    }
    
    // Calibrate
    phi_cal = calibrate_imu();
    
    printf("Running gravity-locked cube...\n");
    last_time = time_us_32();
    
    while (1) {
        // Read IMU
        imu_data_t imu;
        qmi8658_read(&imu);
        
        // Time delta
        uint32_t now = time_us_32();
        float dt = (now - last_time) / 1000000.0f;
        last_time = now;
        
        // Normalize gravity vector
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
        
        // Clear buffer
        clear_buffer(0x0000);
        
        // Reset cube - static orientation
        vertex_t rotated[8] = {
            {-40, -40, -40}, {40, -40, -40}, {40, 40, -40}, {-40, 40, -40},
            {-40, -40,  40}, {40, -40,  40}, {40, 40,  40}, {-40, 40,  40}
        };
        
        // Apply gravity lock rotation (opposite to device rotation)
        for (int i = 0; i < 8; i++) {
            rotate_z(&rotated[i], -phi);
        }
        
        // Draw reference circle
        draw_circle(120, 120, 115, 0x07FF);  // Cyan
        
        // Draw cube
        draw_cube(rotated);
        
        // Draw gravity indicator
        int gx = 120 + (int)(50 * sinf(phi));
        int gy = 120 - (int)(50 * cosf(phi));
        draw_line(120, 120, gx, gy, 0xFFE0);  // Yellow line shows gravity
        
        // Display
        display_buffer();
        
        // Status
        printf("\rAngle: %.1f° | Acc: %.2f %.2f %.2f", 
               phi * 180.0f / PI, imu.ax, imu.ay, imu.az);
        
        // ~20 FPS
        sleep_ms(50);
    }
    
    return 0;
}