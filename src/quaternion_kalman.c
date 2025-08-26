/*
 * Quaternion-based Kalman Filter for Full 3D Orientation
 * 
 * This implementation uses quaternions to avoid gimbal lock and provides
 * smooth, stable orientation tracking in all attitudes.
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "LCD_1in28.h"
#include "DEV_Config.h"

#define WIDTH 240
#define HEIGHT 240
#define CENTER_X 120
#define CENTER_Y 120

// IMU defines
#define QMI8658_ADDR 0x6B
#define QMI8658_CTRL1 0x02
#define QMI8658_CTRL2 0x03
#define QMI8658_CTRL3 0x04
#define QMI8658_CTRL7 0x08
#define QMI8658_AX_L 0x35
#define QMI8658_GX_L 0x3B

#define PI 3.14159265359f
#define TWO_PI 6.28318530718f
#define DEG_TO_RAD (PI / 180.0f)

// Quaternion structure
typedef struct {
    float w, x, y, z;
} quaternion_t;

// Extended Kalman Filter state
typedef struct {
    quaternion_t q;           // Orientation quaternion
    float bias[3];           // Gyro bias estimate
    float P[7][7];          // Error covariance (4 for quat + 3 for bias)
    float Q_gyro;           // Process noise for gyro
    float Q_bias;           // Process noise for bias drift
    float R_accel;          // Measurement noise for accelerometer
} ekf_state_t;

// Frame buffer
static uint16_t frame_buffer[WIDTH * HEIGHT];
static ekf_state_t ekf;
static int display_mode = 0;

// Quaternion operations
void quat_normalize(quaternion_t *q) {
    float norm = sqrtf(q->w*q->w + q->x*q->x + q->y*q->y + q->z*q->z);
    if (norm > 0.0f) {
        q->w /= norm;
        q->x /= norm;
        q->y /= norm;
        q->z /= norm;
    }
}

quaternion_t quat_multiply(const quaternion_t *q1, const quaternion_t *q2) {
    quaternion_t result;
    result.w = q1->w*q2->w - q1->x*q2->x - q1->y*q2->y - q1->z*q2->z;
    result.x = q1->w*q2->x + q1->x*q2->w + q1->y*q2->z - q1->z*q2->y;
    result.y = q1->w*q2->y - q1->x*q2->z + q1->y*q2->w + q1->z*q2->x;
    result.z = q1->w*q2->z + q1->x*q2->y - q1->y*q2->x + q1->z*q2->w;
    return result;
}

// Convert quaternion to rotation matrix
void quat_to_matrix(const quaternion_t *q, float R[3][3]) {
    float w = q->w, x = q->x, y = q->y, z = q->z;
    float xx = x*x, yy = y*y, zz = z*z;
    float xy = x*y, xz = x*z, yz = y*z;
    float wx = w*x, wy = w*y, wz = w*z;
    
    R[0][0] = 1.0f - 2.0f*(yy + zz);
    R[0][1] = 2.0f*(xy - wz);
    R[0][2] = 2.0f*(xz + wy);
    
    R[1][0] = 2.0f*(xy + wz);
    R[1][1] = 1.0f - 2.0f*(xx + zz);
    R[1][2] = 2.0f*(yz - wx);
    
    R[2][0] = 2.0f*(xz - wy);
    R[2][1] = 2.0f*(yz + wx);
    R[2][2] = 1.0f - 2.0f*(xx + yy);
}

// Initialize EKF
void ekf_init(ekf_state_t *ekf) {
    // Initial orientation (identity quaternion)
    ekf->q.w = 1.0f;
    ekf->q.x = 0.0f;
    ekf->q.y = 0.0f;
    ekf->q.z = 0.0f;
    
    // Zero bias
    memset(ekf->bias, 0, sizeof(ekf->bias));
    
    // Initialize covariance
    memset(ekf->P, 0, sizeof(ekf->P));
    for (int i = 0; i < 4; i++) {
        ekf->P[i][i] = 0.01f;  // Small initial uncertainty in quaternion
    }
    for (int i = 4; i < 7; i++) {
        ekf->P[i][i] = 0.001f;  // Small initial uncertainty in bias
    }
    
    // Process noise
    ekf->Q_gyro = 0.001f;   // Gyro noise
    ekf->Q_bias = 0.0001f;  // Bias drift
    
    // Measurement noise
    ekf->R_accel = 0.1f;    // Accelerometer noise
}

// EKF Prediction step
void ekf_predict(ekf_state_t *ekf, float gx, float gy, float gz, float dt) {
    // Correct gyro with bias
    float wx = gx - ekf->bias[0];
    float wy = gy - ekf->bias[1];
    float wz = gz - ekf->bias[2];
    
    // Quaternion derivative
    float qw = ekf->q.w;
    float qx = ekf->q.x;
    float qy = ekf->q.y;
    float qz = ekf->q.z;
    
    // Update quaternion using first-order integration
    float half_dt = 0.5f * dt;
    ekf->q.w += half_dt * (-qx*wx - qy*wy - qz*wz);
    ekf->q.x += half_dt * ( qw*wx + qy*wz - qz*wy);
    ekf->q.y += half_dt * ( qw*wy - qx*wz + qz*wx);
    ekf->q.z += half_dt * ( qw*wz + qx*wy - qy*wx);
    
    // Normalize quaternion
    quat_normalize(&ekf->q);
    
    // Update covariance (simplified - full Jacobian calculation omitted for clarity)
    // In production, you'd compute the full state transition Jacobian
    for (int i = 0; i < 4; i++) {
        ekf->P[i][i] += ekf->Q_gyro * dt * dt;
    }
    for (int i = 4; i < 7; i++) {
        ekf->P[i][i] += ekf->Q_bias * dt * dt;
    }
}

// EKF Update step with accelerometer
void ekf_update_accel(ekf_state_t *ekf, float ax, float ay, float az) {
    // Normalize accelerometer measurement
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm < 0.01f) return;  // Invalid measurement
    
    ax /= norm;
    ay /= norm;
    az /= norm;
    
    // Expected gravity in body frame
    float R[3][3];
    quat_to_matrix(&ekf->q, R);
    
    // Gravity in world frame is [0, 0, -1]
    // Expected measurement is R^T * [0, 0, -1]
    float expected_ax = -R[0][2];
    float expected_ay = -R[1][2];
    float expected_az = -R[2][2];
    
    // Innovation (measurement residual)
    float y[3];
    y[0] = ax - expected_ax;
    y[1] = ay - expected_ay;
    y[2] = az - expected_az;
    
    // Simplified Kalman gain calculation
    // In production, compute full measurement Jacobian
    float K = 0.1f;  // Simplified gain
    
    // Correction quaternion from error
    float correction_angle = sqrtf(y[0]*y[0] + y[1]*y[1] + y[2]*y[2]);
    if (correction_angle > 0.001f) {
        float half_angle = K * correction_angle * 0.5f;
        float s = sinf(half_angle) / correction_angle;
        
        quaternion_t correction;
        correction.w = cosf(half_angle);
        correction.x = s * y[0];
        correction.y = s * y[1];
        correction.z = s * y[2];
        
        // Apply correction
        ekf->q = quat_multiply(&correction, &ekf->q);
        quat_normalize(&ekf->q);
    }
    
    // Update bias estimate (simplified)
    ekf->bias[0] += K * 0.01f * y[0];
    ekf->bias[1] += K * 0.01f * y[1];
    ekf->bias[2] += K * 0.01f * y[2];
    
    // Limit bias
    for (int i = 0; i < 3; i++) {
        if (ekf->bias[i] > 0.5f) ekf->bias[i] = 0.5f;
        if (ekf->bias[i] < -0.5f) ekf->bias[i] = -0.5f;
    }
}

// Vertex structure
typedef struct {
    float x, y, z;
} vertex_t;

// Rotate vertex using quaternion
void rotate_vertex_quat(vertex_t *v, const quaternion_t *q) {
    // Convert vertex to quaternion
    quaternion_t p = {0, v->x, v->y, v->z};
    
    // Conjugate of q
    quaternion_t q_conj = {q->w, -q->x, -q->y, -q->z};
    
    // Rotate: q * p * q^*
    quaternion_t temp = quat_multiply(q, &p);
    quaternion_t result = quat_multiply(&temp, &q_conj);
    
    v->x = result.x;
    v->y = result.y;
    v->z = result.z;
}

// Drawing functions
void clear_buffer(uint16_t color) {
    for (int i = 0; i < WIDTH * HEIGHT; i++) {
        frame_buffer[i] = color;
    }
}

void set_pixel(int x, int y, uint16_t color) {
    if (x >= 0 && x < WIDTH && y >= 0 && y < HEIGHT) {
        frame_buffer[y * WIDTH + x] = color;
    }
}

void draw_line(int x0, int y0, int x1, int y1, uint16_t color) {
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1;
    int sy = y0 < y1 ? 1 : -1;
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

void draw_cube(const vertex_t *vertices, uint16_t color) {
    // Project and draw edges
    int edges[12][2] = {
        {0,1}, {1,2}, {2,3}, {3,0},  // Bottom
        {4,5}, {5,6}, {6,7}, {7,4},  // Top
        {0,4}, {1,5}, {2,6}, {3,7}   // Vertical
    };
    
    for (int i = 0; i < 12; i++) {
        const vertex_t *v0 = &vertices[edges[i][0]];
        const vertex_t *v1 = &vertices[edges[i][1]];
        
        // Simple perspective projection
        float z0 = v0->z + 150.0f;
        float z1 = v1->z + 150.0f;
        
        if (z0 > 10.0f && z1 > 10.0f) {
            int x0 = CENTER_X + (int)(v0->x * 100.0f / z0 * 100.0f);
            int y0 = CENTER_Y + (int)(v0->y * 100.0f / z0 * 100.0f);
            int x1 = CENTER_X + (int)(v1->x * 100.0f / z1 * 100.0f);
            int y1 = CENTER_Y + (int)(v1->y * 100.0f / z1 * 100.0f);
            
            draw_line(x0, y0, x1, y1, color);
        }
    }
}

// Draw orientation vectors
void draw_orientation_vectors(const quaternion_t *q) {
    float R[3][3];
    quat_to_matrix(q, R);
    
    // Draw world axes in body frame
    // X axis (red)
    int x_end = CENTER_X + (int)(R[0][0] * 50);
    int y_end = CENTER_Y + (int)(R[1][0] * 50);
    draw_line(CENTER_X, CENTER_Y, x_end, y_end, 0xF800);
    
    // Y axis (green)
    x_end = CENTER_X + (int)(R[0][1] * 50);
    y_end = CENTER_Y + (int)(R[1][1] * 50);
    draw_line(CENTER_X, CENTER_Y, x_end, y_end, 0x07E0);
    
    // Z axis (blue)
    x_end = CENTER_X + (int)(R[0][2] * 50);
    y_end = CENTER_Y + (int)(R[1][2] * 50);
    draw_line(CENTER_X, CENTER_Y, x_end, y_end, 0x001F);
}

// I2C functions
static void i2c_write_register(uint8_t addr, uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    i2c_write_blocking(i2c1, addr, buf, 2, false);
}

static int16_t i2c_read_16bit(uint8_t addr, uint8_t reg) {
    uint8_t buf[2];
    i2c_write_blocking(i2c1, addr, &reg, 1, true);
    i2c_read_blocking(i2c1, addr, buf, 2, false);
    return (int16_t)(buf[1] << 8 | buf[0]);
}

// IMU initialization
bool qmi8658_init(void) {
    i2c_write_register(QMI8658_ADDR, QMI8658_CTRL1, 0x40);  // 2g, 250Hz
    i2c_write_register(QMI8658_ADDR, QMI8658_CTRL2, 0x44);  // 256dps, 250Hz
    i2c_write_register(QMI8658_ADDR, QMI8658_CTRL3, 0x80);  // Full scale
    i2c_write_register(QMI8658_ADDR, QMI8658_CTRL7, 0x03);  // Enable both
    return true;
}

// Process commands
void process_command(char c) {
    switch(c) {
        case 'r':
            ekf_init(&ekf);
            printf("Reset orientation\n");
            break;
        case 'm':
            display_mode = (display_mode + 1) % 3;
            printf("Display mode: %d\n", display_mode);
            break;
        case '+':
            ekf.R_accel += 0.01f;
            printf("R_accel: %.3f\n", ekf.R_accel);
            break;
        case '-':
            ekf.R_accel -= 0.01f;
            if (ekf.R_accel < 0.01f) ekf.R_accel = 0.01f;
            printf("R_accel: %.3f\n", ekf.R_accel);
            break;
        case 'h':
            printf("\n=== Quaternion EKF Controls ===\n");
            printf("r: Reset orientation\n");
            printf("m: Change display mode\n");
            printf("+/-: Adjust measurement trust\n");
            printf("================================\n");
            break;
    }
}

int main() {
    stdio_init_all();
    sleep_ms(2000);
    
    printf("Quaternion-based EKF starting...\n");
    
    // I2C setup
    i2c_init(i2c1, 400 * 1000);
    gpio_set_function(6, GPIO_FUNC_I2C);
    gpio_set_function(7, GPIO_FUNC_I2C);
    gpio_pull_up(6);
    gpio_pull_up(7);
    
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
    
    // Initialize EKF
    ekf_init(&ekf);
    
    // Calibrate gyro
    printf("Calibrating gyro...\n");
    float gx_bias = 0, gy_bias = 0, gz_bias = 0;
    for (int i = 0; i < 200; i++) {
        int16_t raw_gx = i2c_read_16bit(QMI8658_ADDR, QMI8658_GX_L);
        int16_t raw_gy = i2c_read_16bit(QMI8658_ADDR, QMI8658_GX_L + 2);
        int16_t raw_gz = i2c_read_16bit(QMI8658_ADDR, QMI8658_GX_L + 4);
        gx_bias += raw_gx / 128.0f;
        gy_bias += raw_gy / 128.0f;
        gz_bias += raw_gz / 128.0f;
        sleep_ms(5);
    }
    gx_bias = (gx_bias / 200.0f) * DEG_TO_RAD;
    gy_bias = (gy_bias / 200.0f) * DEG_TO_RAD;
    gz_bias = (gz_bias / 200.0f) * DEG_TO_RAD;
    
    // Set initial bias estimate
    ekf.bias[0] = gx_bias;
    ekf.bias[1] = gy_bias;
    ekf.bias[2] = gz_bias;
    
    printf("Calibration done. Running...\n");
    printf("Press 'h' for help\n");
    
    // Define cube vertices
    vertex_t cube[8] = {
        {-0.5f, -0.5f, -0.5f}, {0.5f, -0.5f, -0.5f},
        {0.5f, 0.5f, -0.5f}, {-0.5f, 0.5f, -0.5f},
        {-0.5f, -0.5f, 0.5f}, {0.5f, -0.5f, 0.5f},
        {0.5f, 0.5f, 0.5f}, {-0.5f, 0.5f, 0.5f}
    };
    
    uint32_t last_time = time_us_32();
    
    while (1) {
        // Check for commands
        int c = getchar_timeout_us(0);
        if (c != PICO_ERROR_TIMEOUT) {
            process_command((char)c);
        }
        
        // Read sensors
        int16_t raw_ax = i2c_read_16bit(QMI8658_ADDR, QMI8658_AX_L);
        int16_t raw_ay = i2c_read_16bit(QMI8658_ADDR, QMI8658_AX_L + 2);
        int16_t raw_az = i2c_read_16bit(QMI8658_ADDR, QMI8658_AX_L + 4);
        
        int16_t raw_gx = i2c_read_16bit(QMI8658_ADDR, QMI8658_GX_L);
        int16_t raw_gy = i2c_read_16bit(QMI8658_ADDR, QMI8658_GX_L + 2);
        int16_t raw_gz = i2c_read_16bit(QMI8658_ADDR, QMI8658_GX_L + 4);
        
        // Convert to physical units with axis corrections
        float ax = raw_ay / 16384.0f;  // Swapped X/Y
        float ay = raw_ax / 16384.0f;
        float az = raw_az / 16384.0f;
        
        float gx = (raw_gx / 128.0f) * DEG_TO_RAD;
        float gy = -(raw_gy / 128.0f) * DEG_TO_RAD;  // Inverted
        float gz = -(raw_gz / 128.0f) * DEG_TO_RAD;  // Inverted
        
        // Calculate dt
        uint32_t now = time_us_32();
        float dt = (now - last_time) / 1000000.0f;
        if (dt > 0.1f) dt = 0.01f;
        last_time = now;
        
        // Run EKF
        ekf_predict(&ekf, gx, gy, gz, dt);
        ekf_update_accel(&ekf, ax, ay, az);
        
        // Clear display
        clear_buffer(0x0000);
        
        // Rotate and draw cube
        vertex_t rotated_cube[8];
        for (int i = 0; i < 8; i++) {
            rotated_cube[i] = cube[i];
            rotate_vertex_quat(&rotated_cube[i], &ekf.q);
        }
        draw_cube(rotated_cube, 0x07E0);
        
        // Draw orientation vectors if requested
        if (display_mode == 1 || display_mode == 2) {
            draw_orientation_vectors(&ekf.q);
        }
        
        // Draw gravity vector in body frame
        if (display_mode == 2) {
            float R[3][3];
            quat_to_matrix(&ekf.q, R);
            
            // Gravity in body frame
            float grav_x = -R[0][2] * 40;
            float grav_y = -R[1][2] * 40;
            
            draw_line(CENTER_X, CENTER_Y, 
                     CENTER_X + (int)grav_x, 
                     CENTER_Y + (int)grav_y, 
                     0xFFFF);
        }
        
        // Update display
        LCD_1IN28_Display(frame_buffer);
        
        sleep_ms(10);
    }
    
    return 0;
}