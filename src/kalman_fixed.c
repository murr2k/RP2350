/**
 * Fixed Kalman Filter with Proper Angle Handling
 * Addresses discontinuous flipping and angle wrapping issues
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
#define TWO_PI (2.0f * PI)
#define RAD_TO_DEG (180.0f / PI)
#define DEG_TO_RAD (PI / 180.0f)
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

// Simple Kalman for single axis (more stable than 2D)
typedef struct {
    float angle;      // Current angle estimate
    float bias;       // Gyro bias estimate
    float P[2][2];    // Error covariance matrix
    
    // Tunable parameters
    float Q_angle;    // Process noise variance for angle
    float Q_bias;     // Process noise variance for bias  
    float R_measure;  // Measurement noise variance
} kalman_t;

static kalman_t kalman_pitch;
static kalman_t kalman_roll;
static float kalman_yaw = 0;  // Simple integrated yaw for Kalman

// Complementary filter for comparison/backup
static float comp_pitch = 0;
static float comp_roll = 0;
static float comp_yaw = 0;

// Display mode
static int display_mode = 0;  // 0=Kalman, 1=Complementary, 2=Both

// Initialize Kalman
void kalman_init(kalman_t *k) {
    k->angle = 0.0f;
    k->bias = 0.0f;
    
    // Initialize covariance matrix
    k->P[0][0] = 1.0f;
    k->P[0][1] = 0.0f;
    k->P[1][0] = 0.0f;  
    k->P[1][1] = 1.0f;
    
    // Conservative tuning to prevent instability
    k->Q_angle = 0.001f;   // Angle process noise
    k->Q_bias = 0.003f;    // Bias process noise
    k->R_measure = 0.5f;   // Measurement noise (higher = trust gyro more)
}

// Wrap angle to [-PI, PI]
float wrap_angle(float angle) {
    while (angle > PI) angle -= TWO_PI;
    while (angle < -PI) angle += TWO_PI;
    return angle;
}

// Angle difference handling discontinuities
float angle_diff(float a, float b) {
    float diff = a - b;
    if (diff > PI) diff -= TWO_PI;
    if (diff < -PI) diff += TWO_PI;
    return diff;
}

// Single-axis Kalman update with proper angle wrapping
float kalman_update(kalman_t *k, float rate, float angle_measured, float dt) {
    // --- Prediction step ---
    // Update angle from gyro
    k->angle += dt * (rate - k->bias);
    k->angle = wrap_angle(k->angle);
    
    // Update error covariance
    k->P[0][0] += dt * (dt * k->P[1][1] - k->P[0][1] - k->P[1][0] + k->Q_angle);
    k->P[0][1] -= dt * k->P[1][1];
    k->P[1][0] -= dt * k->P[1][1];
    k->P[1][1] += k->Q_bias * dt;
    
    // --- Update step ---
    // Calculate innovation with proper angle wrapping
    float innovation = angle_diff(angle_measured, k->angle);
    
    // Innovation covariance
    float S = k->P[0][0] + k->R_measure;
    
    // Kalman gain
    float K[2];
    K[0] = k->P[0][0] / S;
    K[1] = k->P[1][0] / S;
    
    // Update state with innovation
    k->angle += K[0] * innovation;
    k->angle = wrap_angle(k->angle);
    k->bias += K[1] * innovation;
    
    // Limit bias to reasonable range
    if (k->bias > 0.5f) k->bias = 0.5f;
    if (k->bias < -0.5f) k->bias = -0.5f;
    
    // Update error covariance
    float P00_temp = k->P[0][0];
    float P01_temp = k->P[0][1];
    
    k->P[0][0] -= K[0] * P00_temp;
    k->P[0][1] -= K[0] * P01_temp;
    k->P[1][0] -= K[1] * P00_temp;
    k->P[1][1] -= K[1] * P01_temp;
    
    return k->angle;
}

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
    i2c_write_register(QMI8658_ADDR, QMI8658_CTRL7, 0x03);
    
    return true;
}

// Cube vertices
typedef struct {
    float x, y, z;
} vertex_t;

vertex_t cube[8] = {
    {-50, -50, -50}, {50, -50, -50}, {50, 50, -50}, {-50, 50, -50},
    {-50, -50,  50}, {50, -50,  50}, {50, 50,  50}, {-50, 50,  50}
};

const int edges[12][2] = {
    {0,1}, {1,2}, {2,3}, {3,0},
    {4,5}, {5,6}, {6,7}, {7,4},
    {0,4}, {1,5}, {2,6}, {3,7}
};

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

// 3D rotation
void rotate_xyz(vertex_t *v, float pitch, float roll, float yaw) {
    float cos_p = cosf(pitch), sin_p = sinf(pitch);
    float cos_r = cosf(roll), sin_r = sinf(roll);
    float cos_y = cosf(yaw), sin_y = sinf(yaw);
    
    float x = v->x;
    float y = v->y;
    float z = v->z;
    
    // Yaw
    float x1 = x * cos_y - y * sin_y;
    float y1 = x * sin_y + y * cos_y;
    float z1 = z;
    
    // Pitch
    float x2 = x1 * cos_p + z1 * sin_p;
    float y2 = y1;
    float z2 = -x1 * sin_p + z1 * cos_p;
    
    // Roll
    v->x = x2;
    v->y = y2 * cos_r - z2 * sin_r;
    v->z = y2 * sin_r + z2 * cos_r;
}

// Project 3D to 2D
void project(vertex_t v, int *x, int *y) {
    float distance = 200.0f;
    float z = v.z + distance;
    *x = (int)(120 + (v.x * 120) / z);
    *y = (int)(120 + (v.y * 120) / z);
}

// Draw cube
void draw_cube(vertex_t *vertices, uint16_t color) {
    int screen[8][2];
    
    for (int i = 0; i < 8; i++) {
        project(vertices[i], &screen[i][0], &screen[i][1]);
    }
    
    for (int i = 0; i < 12; i++) {
        draw_line(screen[edges[i][0]][0], screen[edges[i][0]][1],
                 screen[edges[i][1]][0], screen[edges[i][1]][1], color);
    }
}

// Draw angle indicator
void draw_angle_bar(int x, int y, float angle, uint16_t color) {
    int bar_width = (int)(angle * RAD_TO_DEG);
    if (bar_width > 45) bar_width = 45;
    if (bar_width < -45) bar_width = -45;
    
    // Draw baseline
    draw_line(x - 50, y, x + 50, y, 0x4208);
    
    // Draw angle bar
    if (bar_width > 0) {
        for (int i = 0; i < bar_width; i++) {
            draw_line(x, y - 2, x + i, y + 2, color);
        }
    } else {
        for (int i = 0; i > bar_width; i--) {
            draw_line(x, y - 2, x + i, y + 2, color);
        }
    }
    
    // Draw center mark
    draw_line(x, y - 5, x, y + 5, 0xFFFF);
}

// Process command
void process_command(char c) {
    switch(c) {
        case 'm': 
            display_mode = (display_mode + 1) % 3;
            printf("Mode: %s\n", 
                   display_mode == 0 ? "Kalman" : 
                   display_mode == 1 ? "Complementary" : "Both");
            break;
        case '+':
            kalman_pitch.R_measure += 0.1f;
            kalman_roll.R_measure += 0.1f;
            printf("R_measure: %.2f (higher = trust gyro more)\n", kalman_pitch.R_measure);
            break;
        case '-':
            kalman_pitch.R_measure -= 0.1f;
            kalman_roll.R_measure -= 0.1f;
            if (kalman_pitch.R_measure < 0.1f) kalman_pitch.R_measure = 0.1f;
            if (kalman_roll.R_measure < 0.1f) kalman_roll.R_measure = 0.1f;
            printf("R_measure: %.2f\n", kalman_pitch.R_measure);
            break;
        case 'r':
            kalman_pitch.angle = 0;
            kalman_pitch.bias = 0;
            kalman_roll.angle = 0;
            kalman_roll.bias = 0;
            kalman_yaw = 0;
            comp_pitch = 0;
            comp_roll = 0;
            comp_yaw = 0;
            printf("Reset angles\n");
            break;
        case 'h':
            printf("\n=== Controls ===\n");
            printf("m: Switch display mode\n");
            printf("+/-: Adjust Kalman R_measure\n");
            printf("r: Reset angles\n");
            printf("h: Show this help\n");
            break;
    }
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    
    printf("\n=== Fixed Kalman Filter Demo ===\n");
    printf("Press 'h' for help\n\n");
    
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
    
    // Initialize filters
    kalman_init(&kalman_pitch);
    kalman_init(&kalman_roll);
    kalman_yaw = 0;  // Reset yaw
    
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
    
    printf("Calibration done. Running...\n");
    
    uint32_t last_time = time_us_32();
    const float alpha = 0.98f;  // Complementary filter constant
    
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
        
        // Convert to physical units
        float ax = raw_ax / 16384.0f;
        float ay = raw_ay / 16384.0f;
        float az = raw_az / 16384.0f;
        
        // Gyro with bias correction and axis fix
        float gx = (raw_gx / 128.0f) * DEG_TO_RAD - gx_bias;
        float gy = -(raw_gy / 128.0f) * DEG_TO_RAD - gy_bias;  // Inverted
        float gz = -(raw_gz / 128.0f) * DEG_TO_RAD - gz_bias;  // Inverted
        
        // Calculate dt
        uint32_t now = time_us_32();
        float dt = (now - last_time) / 1000000.0f;
        if (dt > 0.1f) dt = 0.01f;  // Limit first iteration
        last_time = now;
        
        // Calculate angles from accelerometer (swapped X/Y)
        // Use atan2 for full range without discontinuities
        float accel_pitch = atan2f(ay, sqrtf(ax * ax + az * az));  // Now from Y accel
        float accel_roll = atan2f(ax, sqrtf(ay * ay + az * az));   // Now from X accel
        
        // Sanity check - if accelerometer magnitude is way off, don't trust it
        float accel_mag = sqrtf(ax * ax + ay * ay + az * az);
        bool accel_valid = (accel_mag > 0.8f && accel_mag < 1.2f);
        
        // Update Kalman filters only with valid accelerometer (swapped X/Y)
        float kalman_pitch_angle, kalman_roll_angle;
        if (accel_valid) {
            kalman_pitch_angle = kalman_update(&kalman_pitch, gx, accel_pitch, dt);  // gx with pitch
            kalman_roll_angle = kalman_update(&kalman_roll, gy, accel_roll, dt);     // gy with roll
        } else {
            // Gyro only update (swapped)
            kalman_pitch.angle += gx * dt;  // gx for pitch
            kalman_roll.angle += gy * dt;   // gy for roll
            kalman_pitch_angle = wrap_angle(kalman_pitch.angle);
            kalman_roll_angle = wrap_angle(kalman_roll.angle);
        }
        
        // Update yaw (no absolute reference, just integrate)
        kalman_yaw += gz * dt;
        kalman_yaw = wrap_angle(kalman_yaw);
        
        // Complementary filter for comparison (swapped X/Y)
        if (accel_valid) {
            comp_pitch = alpha * (comp_pitch + gx * dt) + (1.0f - alpha) * accel_pitch;  // gx with pitch
            comp_roll = alpha * (comp_roll + gy * dt) + (1.0f - alpha) * accel_roll;     // gy with roll
        } else {
            comp_pitch += gx * dt;  // gx for pitch
            comp_roll += gy * dt;   // gy for roll
        }
        comp_pitch = wrap_angle(comp_pitch);
        comp_roll = wrap_angle(comp_roll);
        comp_yaw += gz * dt;
        comp_yaw = wrap_angle(comp_yaw);
        
        // Clear display
        clear_buffer(0x0000);
        
        // Draw based on mode
        if (display_mode == 0 || display_mode == 2) {
            // Kalman cube
            vertex_t kalman_cube[8];
            for (int i = 0; i < 8; i++) {
                kalman_cube[i] = cube[i];
                rotate_xyz(&kalman_cube[i], kalman_pitch_angle, kalman_roll_angle, kalman_yaw);
            }
            draw_cube(kalman_cube, 0x07E0);  // Green for Kalman
            
            // Kalman angles
            draw_angle_bar(120, 200, kalman_pitch_angle, 0x07E0);
            draw_angle_bar(120, 210, kalman_roll_angle, 0x07E0);
        }
        
        if (display_mode == 1 || display_mode == 2) {
            // Complementary cube
            vertex_t comp_cube[8];
            for (int i = 0; i < 8; i++) {
                comp_cube[i] = cube[i];
                if (display_mode == 2) {
                    // Offset for comparison
                    comp_cube[i].x *= 0.5f;
                    comp_cube[i].y *= 0.5f;
                    comp_cube[i].z *= 0.5f;
                }
                rotate_xyz(&comp_cube[i], comp_pitch, comp_roll, comp_yaw);
            }
            draw_cube(comp_cube, 0xF800);  // Red for complementary
            
            // Comp angles
            if (display_mode == 1) {
                draw_angle_bar(120, 200, comp_pitch, 0xF800);
                draw_angle_bar(120, 210, comp_roll, 0xF800);
            }
        }
        
        // Status indicator
        if (!accel_valid) {
            // Draw warning if accelerometer is invalid
            for (int i = 0; i < 10; i++) {
                draw_line(10 + i, 10, 10 + i, 20, 0xF800);
            }
        }
        
        // Display
        display_buffer();
        
        // Debug output
        printf("\rK: P:%+6.2f R:%+6.2f | C: P:%+6.2f R:%+6.2f | Bias: %+.3f %+.3f | %s",
               kalman_pitch_angle * RAD_TO_DEG, kalman_roll_angle * RAD_TO_DEG,
               comp_pitch * RAD_TO_DEG, comp_roll * RAD_TO_DEG,
               kalman_pitch.bias, kalman_roll.bias,
               accel_valid ? "OK" : "!!");
        
        sleep_ms(10);  // 100Hz update
    }
    
    return 0;
}