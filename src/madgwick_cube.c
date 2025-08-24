/**
 * Madgwick Filter 6-DOF Cube Demo
 * Uses Madgwick's gradient descent algorithm for sensor fusion
 * Provides drift-free orientation from accelerometer + gyroscope
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

// Madgwick filter parameters
#define SAMPLE_RATE 100.0f  // Hz
#define BETA 0.1f           // Madgwick filter gain

// Frame buffer
static uint16_t frame_buffer[WIDTH * HEIGHT];

// Quaternion representing orientation
typedef struct {
    float w, x, y, z;
} quaternion_t;

// Global orientation quaternion
static quaternion_t q = {1.0f, 0.0f, 0.0f, 0.0f};

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

// Face definitions for filled rendering (optional)
const int faces[6][4] = {
    {0, 1, 2, 3},  // Back
    {4, 7, 6, 5},  // Front
    {0, 3, 7, 4},  // Left
    {1, 5, 6, 2},  // Right
    {3, 2, 6, 7},  // Top
    {0, 4, 5, 1}   // Bottom
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

// Read IMU data
static void read_imu(float *ax, float *ay, float *az, float *gx, float *gy, float *gz) {
    // Read accelerometer
    int16_t raw_ax = i2c_read_16bit(QMI8658_ADDR, QMI8658_AX_L);
    int16_t raw_ay = i2c_read_16bit(QMI8658_ADDR, QMI8658_AX_L + 2);
    int16_t raw_az = i2c_read_16bit(QMI8658_ADDR, QMI8658_AX_L + 4);
    
    // Read gyroscope
    int16_t raw_gx = i2c_read_16bit(QMI8658_ADDR, QMI8658_GX_L);
    int16_t raw_gy = i2c_read_16bit(QMI8658_ADDR, QMI8658_GX_L + 2);
    int16_t raw_gz = i2c_read_16bit(QMI8658_ADDR, QMI8658_GX_L + 4);
    
    // Convert to physical units
    // Based on rotation_test results:
    // - X (roll) needs to be inverted
    // - Y (pitch) is correct
    // - Z (yaw) is correct
    *ax = raw_ax / 16384.0f;  // 2g range
    *ay = raw_ay / 16384.0f;
    *az = raw_az / 16384.0f;
    
    // Convert to rad/s with proper mapping
    *gx = (raw_gx / 128.0f) * (PI / 180.0f);   // Don't invert X (was wrong)
    *gy = -(raw_gy / 128.0f) * (PI / 180.0f);  // Invert Y (pitch)
    *gz = (raw_gz / 128.0f) * (PI / 180.0f);   // Z is correct (yaw)
}

// Fast inverse square root (using union to avoid aliasing)
float invSqrt(float x) {
    union {
        float f;
        uint32_t i;
    } conv;
    
    float halfx = 0.5f * x;
    conv.f = x;
    conv.i = 0x5f3759df - (conv.i >> 1);
    conv.f = conv.f * (1.5f - (halfx * conv.f * conv.f));
    return conv.f;
}

// Madgwick filter update
void madgwick_update(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
    
    // Gyro is already in rad/s from read_imu()
    
    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q.x * gx - q.y * gy - q.z * gz);
    qDot2 = 0.5f * (q.w * gx + q.y * gz - q.z * gy);
    qDot3 = 0.5f * (q.w * gy - q.x * gz + q.z * gx);
    qDot4 = 0.5f * (q.w * gz + q.x * gy - q.y * gx);
    
    // Compute feedback only if accelerometer measurement valid
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // Normalize accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;
        
        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * q.w;
        _2q1 = 2.0f * q.x;
        _2q2 = 2.0f * q.y;
        _2q3 = 2.0f * q.z;
        _4q0 = 4.0f * q.w;
        _4q1 = 4.0f * q.x;
        _4q2 = 4.0f * q.y;
        _8q1 = 8.0f * q.x;
        _8q2 = 8.0f * q.y;
        q0q0 = q.w * q.w;
        q1q1 = q.x * q.x;
        q2q2 = q.y * q.y;
        q3q3 = q.z * q.z;
        
        // Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q.x - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q.y + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q.z - _2q1 * ax + 4.0f * q2q2 * q.z - _2q2 * ay;
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;
        
        // Apply feedback step
        qDot1 -= BETA * s0;
        qDot2 -= BETA * s1;
        qDot3 -= BETA * s2;
        qDot4 -= BETA * s3;
    }
    
    // Integrate rate of change of quaternion to yield quaternion
    q.w += qDot1 * dt;
    q.x += qDot2 * dt;
    q.y += qDot3 * dt;
    q.z += qDot4 * dt;
    
    // Normalize quaternion
    recipNorm = invSqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    q.w *= recipNorm;
    q.x *= recipNorm;
    q.y *= recipNorm;
    q.z *= recipNorm;
}

// Rotate vertex using quaternion
vertex_t rotate_by_quaternion(vertex_t v, quaternion_t q) {
    // Convert vertex to quaternion
    quaternion_t p = {0, v.x, v.y, v.z};
    
    // q * p * q^-1
    quaternion_t q_conj = {q.w, -q.x, -q.y, -q.z};
    
    // First: q * p
    quaternion_t qp;
    qp.w = q.w * p.w - q.x * p.x - q.y * p.y - q.z * p.z;
    qp.x = q.w * p.x + q.x * p.w + q.y * p.z - q.z * p.y;
    qp.y = q.w * p.y - q.x * p.z + q.y * p.w + q.z * p.x;
    qp.z = q.w * p.z + q.x * p.y - q.y * p.x + q.z * p.w;
    
    // Second: (q * p) * q^-1
    quaternion_t result;
    result.w = qp.w * q_conj.w - qp.x * q_conj.x - qp.y * q_conj.y - qp.z * q_conj.z;
    result.x = qp.w * q_conj.x + qp.x * q_conj.w + qp.y * q_conj.z - qp.z * q_conj.y;
    result.y = qp.w * q_conj.y - qp.x * q_conj.z + qp.y * q_conj.w + qp.z * q_conj.x;
    result.z = qp.w * q_conj.z + qp.x * q_conj.y - qp.y * q_conj.x + qp.z * q_conj.w;
    
    vertex_t rotated = {result.x, result.y, result.z};
    return rotated;
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
    
    // Draw edges
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

// Draw orientation axes
void draw_axes(quaternion_t orientation) {
    // Define axis endpoints in object space
    vertex_t x_axis = {40, 0, 0};
    vertex_t y_axis = {0, 40, 0};
    vertex_t z_axis = {0, 0, 40};
    vertex_t origin = {0, 0, 0};
    
    // Rotate axes by orientation
    vertex_t x_rot = rotate_by_quaternion(x_axis, orientation);
    vertex_t y_rot = rotate_by_quaternion(y_axis, orientation);
    vertex_t z_rot = rotate_by_quaternion(z_axis, orientation);
    
    // Project to screen
    int ox, oy, xx, xy, yx, yy, zx, zy;
    project(origin, &ox, &oy);
    project(x_rot, &xx, &xy);
    project(y_rot, &yx, &yy);
    project(z_rot, &zx, &zy);
    
    // Draw axes
    draw_line(ox, oy, xx, xy, 0xF800);  // X = Red
    draw_line(ox, oy, yx, yy, 0x07E0);  // Y = Green
    draw_line(ox, oy, zx, zy, 0x001F);  // Z = Blue
}

// Convert quaternion to Euler angles for display
void quaternion_to_euler(quaternion_t q, float *roll, float *pitch, float *yaw) {
    // Roll (x-axis rotation)
    float sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    *roll = atan2f(sinr_cosp, cosr_cosp);
    
    // Pitch (y-axis rotation)
    float sinp = 2 * (q.w * q.y - q.z * q.x);
    if (fabsf(sinp) >= 1)
        *pitch = copysignf(PI / 2, sinp);
    else
        *pitch = asinf(sinp);
    
    // Yaw (z-axis rotation)
    float siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    *yaw = atan2f(siny_cosp, cosy_cosp);
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    
    printf("\n=== Madgwick 6-DOF Cube ===\n");
    printf("Full orientation tracking with Madgwick filter\n\n");
    
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
    
    printf("Calibrating... hold still for 2 seconds\n");
    
    // Calibrate gyro bias
    float gx_bias = 0, gy_bias = 0, gz_bias = 0;
    const int cal_samples = 200;
    for (int i = 0; i < cal_samples; i++) {
        float ax, ay, az, gx, gy, gz;
        read_imu(&ax, &ay, &az, &gx, &gy, &gz);
        gx_bias += gx;
        gy_bias += gy;
        gz_bias += gz;
        sleep_ms(10);
    }
    gx_bias /= cal_samples;
    gy_bias /= cal_samples;
    gz_bias /= cal_samples;
    
    printf("Calibration complete. Running Madgwick filter...\n");
    
    uint32_t last_time = time_us_32();
    
    while (1) {
        // Read IMU
        float ax, ay, az, gx, gy, gz;
        read_imu(&ax, &ay, &az, &gx, &gy, &gz);
        
        // Remove gyro bias
        gx -= gx_bias;
        gy -= gy_bias;
        gz -= gz_bias;
        
        // Calculate dt
        uint32_t now = time_us_32();
        float dt = (now - last_time) / 1000000.0f;
        last_time = now;
        
        // Update Madgwick filter
        madgwick_update(gx, gy, gz, ax, ay, az, dt);
        
        // Clear buffer
        clear_buffer(0x0000);
        
        // Create rotated cube
        vertex_t rotated[8];
        for (int i = 0; i < 8; i++) {
            rotated[i] = rotate_by_quaternion(cube[i], q);
        }
        
        // Draw coordinate axes
        draw_axes(q);
        
        // Draw cube
        draw_cube(rotated);
        
        // Get Euler angles for display
        float roll, pitch, yaw;
        quaternion_to_euler(q, &roll, &pitch, &yaw);
        
        // Display
        display_buffer();
        
        // Debug output
        printf("\rR:%6.1f° P:%6.1f° Y:%6.1f° | Q: %.2f %.2f %.2f %.2f", 
               roll * 180.0f / PI, pitch * 180.0f / PI, yaw * 180.0f / PI,
               q.w, q.x, q.y, q.z);
        
        // Target ~100 Hz update rate
        sleep_ms(10);
    }
    
    return 0;
}