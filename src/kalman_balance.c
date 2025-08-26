/**
 * Kalman Filter Balance Demo
 * High-stability orientation tracking suitable for balancing robots
 * Optimized for 2-axis stability with drift-free performance
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

// Kalman filter update rate
#define DT 0.005f  // 200Hz for maximum stability

// Frame buffer
static uint16_t frame_buffer[WIDTH * HEIGHT];

// Kalman Filter State Structure (2D for pitch/roll)
typedef struct {
    // State vector: [angle, gyro_bias]
    float angle;      // Filtered angle (rad)
    float bias;       // Gyro bias (rad/s)
    float rate;       // Unbiased rotation rate
    
    // Covariance matrix (2x2)
    float P[2][2];    // Error covariance
    
    // Process noise covariance
    float Q_angle;    // Process noise for angle
    float Q_bias;     // Process noise for bias
    
    // Measurement noise covariance
    float R_measure;  // Measurement noise
} kalman_state_t;

// Two Kalman filters for pitch and roll
static kalman_state_t kalman_pitch;
static kalman_state_t kalman_roll;

// Additional state for yaw (complementary filter since no magnetometer)
static float yaw = 0;
static float yaw_rate = 0;

// Balance metrics
static float balance_angle_pitch = 0;
static float balance_angle_roll = 0;
static float stability_score = 100.0f;

// Cube vertices for visualization
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

// Initialize Kalman filter
void kalman_init(kalman_state_t *state) {
    // Initial state
    state->angle = 0.0f;
    state->bias = 0.0f;
    state->rate = 0.0f;
    
    // Initial covariance (high uncertainty)
    state->P[0][0] = 1.0f;
    state->P[0][1] = 0.0f;
    state->P[1][0] = 0.0f;
    state->P[1][1] = 1.0f;
    
    // Process noise - tune these for your application
    // Lower values = trust gyro more (less responsive to changes)
    // Higher values = trust accelerometer more (more responsive but noisier)
    state->Q_angle = 0.001f;  // Very low - trust gyro for angle
    state->Q_bias = 0.003f;   // Low - gyro bias changes slowly
    
    // Measurement noise - tune based on accelerometer quality
    state->R_measure = 0.03f;  // Moderate - accelerometer has some noise
}

// Kalman filter update
float kalman_update(kalman_state_t *state, float gyro_rate, float accel_angle, float dt) {
    // --- Prediction Step ---
    // State prediction
    // angle = angle + dt * (gyro_rate - bias)
    state->rate = gyro_rate - state->bias;
    state->angle += dt * state->rate;
    
    // Error covariance prediction
    // P = A * P * A' + Q
    // where A = [1, -dt; 0, 1]
    state->P[0][0] += dt * (dt * state->P[1][1] - state->P[0][1] - state->P[1][0] + state->Q_angle);
    state->P[0][1] -= dt * state->P[1][1];
    state->P[1][0] -= dt * state->P[1][1];
    state->P[1][1] += state->Q_bias * dt;
    
    // --- Update Step ---
    // Innovation (measurement residual)
    float innovation = accel_angle - state->angle;
    
    // Innovation covariance
    float S = state->P[0][0] + state->R_measure;
    
    // Kalman gain
    float K[2];
    K[0] = state->P[0][0] / S;
    K[1] = state->P[1][0] / S;
    
    // State update
    state->angle += K[0] * innovation;
    state->bias += K[1] * innovation;
    
    // Error covariance update
    float P00_temp = state->P[0][0];
    float P01_temp = state->P[0][1];
    
    state->P[0][0] -= K[0] * P00_temp;
    state->P[0][1] -= K[0] * P01_temp;
    state->P[1][0] -= K[1] * P00_temp;
    state->P[1][1] -= K[1] * P01_temp;
    
    return state->angle;
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
    
    // Reset
    i2c_write_register(QMI8658_ADDR, QMI8658_CTRL1, 0x60);
    sleep_ms(10);
    
    // Configure for high stability
    // Accelerometer: 2g range, 500Hz for oversampling
    i2c_write_register(QMI8658_ADDR, QMI8658_CTRL2, 0x05);  
    
    // Gyroscope: 256 dps range, 500Hz
    i2c_write_register(QMI8658_ADDR, QMI8658_CTRL3, 0x55);
    
    // Enable sensors
    i2c_write_register(QMI8658_ADDR, QMI8658_CTRL7, 0x03);
    
    return true;
}

// Read and process IMU data
void read_imu_filtered(float *pitch_angle, float *roll_angle, float *yaw_angle) {
    static uint32_t last_time = 0;
    static float ax_filtered = 0, ay_filtered = 0, az_filtered = 1.0f;
    const float alpha = 0.02f;  // Low-pass filter for accelerometer
    
    // Read raw values
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
    
    // Gyro in rad/s (with correct axis mapping from earlier testing)
    float gx = (raw_gx / 128.0f) * DEG_TO_RAD;
    float gy = -(raw_gy / 128.0f) * DEG_TO_RAD;  // Inverted per testing
    float gz = (raw_gz / 128.0f) * DEG_TO_RAD;
    
    // Apply low-pass filter to accelerometer
    ax_filtered = alpha * ax + (1.0f - alpha) * ax_filtered;
    ay_filtered = alpha * ay + (1.0f - alpha) * ay_filtered;
    az_filtered = alpha * az + (1.0f - alpha) * az_filtered;
    
    // Calculate angles from accelerometer
    float accel_pitch = atan2f(ax_filtered, sqrtf(ay_filtered * ay_filtered + az_filtered * az_filtered));
    float accel_roll = atan2f(-ay_filtered, az_filtered);
    
    // Get time delta
    uint32_t now = time_us_32();
    float dt = (now - last_time) / 1000000.0f;
    if (dt > 0.1f) dt = DT;  // Limit max dt on first iteration
    last_time = now;
    
    // Update Kalman filters
    *pitch_angle = kalman_update(&kalman_pitch, gy, accel_pitch, dt);
    *roll_angle = kalman_update(&kalman_roll, gx, accel_roll, dt);
    
    // Simple yaw integration (no magnetometer)
    yaw_rate = gz;
    yaw += gz * dt;
    if (yaw > PI) yaw -= 2 * PI;
    if (yaw < -PI) yaw += 2 * PI;
    *yaw_angle = yaw;
    
    // Update balance metrics
    balance_angle_pitch = *pitch_angle;
    balance_angle_roll = *roll_angle;
    
    // Calculate stability score (0-100)
    float pitch_error = fabsf(*pitch_angle) * RAD_TO_DEG;
    float roll_error = fabsf(*roll_angle) * RAD_TO_DEG;
    float rate_error = (fabsf(kalman_pitch.rate) + fabsf(kalman_roll.rate)) * RAD_TO_DEG;
    
    stability_score = 100.0f - fminf(100.0f, pitch_error * 2 + roll_error * 2 + rate_error);
    if (stability_score < 0) stability_score = 0;
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

// Draw filled rectangle
void fill_rect(int x, int y, int w, int h, uint16_t color) {
    for (int i = y; i < y + h && i < HEIGHT; i++) {
        for (int j = x; j < x + w && j < WIDTH; j++) {
            set_pixel(j, i, color);
        }
    }
}

// 3D rotation functions
void rotate_xyz(vertex_t *v, float pitch, float roll, float yaw) {
    // Apply rotations in order: yaw, pitch, roll
    float cos_p = cosf(pitch), sin_p = sinf(pitch);
    float cos_r = cosf(roll), sin_r = sinf(roll);
    float cos_y = cosf(yaw), sin_y = sinf(yaw);
    
    // Combined rotation matrix multiplication
    float x = v->x;
    float y = v->y;
    float z = v->z;
    
    // Yaw rotation (around Z)
    float x1 = x * cos_y - y * sin_y;
    float y1 = x * sin_y + y * cos_y;
    float z1 = z;
    
    // Pitch rotation (around Y)
    float x2 = x1 * cos_p + z1 * sin_p;
    float y2 = y1;
    float z2 = -x1 * sin_p + z1 * cos_p;
    
    // Roll rotation (around X)
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

// Draw wireframe cube
void draw_cube(vertex_t *vertices) {
    int screen[8][2];
    
    for (int i = 0; i < 8; i++) {
        project(vertices[i], &screen[i][0], &screen[i][1]);
    }
    
    for (int i = 0; i < 12; i++) {
        int v0 = edges[i][0];
        int v1 = edges[i][1];
        
        // Color based on stability
        uint16_t color = 0xFFFF;
        if (stability_score > 80) color = 0x07E0;  // Green - stable
        else if (stability_score > 50) color = 0xFFE0;  // Yellow - moderate
        else color = 0xF800;  // Red - unstable
        
        draw_line(screen[v0][0], screen[v0][1], 
                 screen[v1][0], screen[v1][1], color);
    }
}

// Draw balance indicator
void draw_balance_indicator(void) {
    // Draw horizon line
    int horizon_y = 120 - (int)(balance_angle_pitch * RAD_TO_DEG * 2);
    
    // Clamp to screen
    if (horizon_y < 0) horizon_y = 0;
    if (horizon_y >= HEIGHT) horizon_y = HEIGHT - 1;
    
    draw_line(20, horizon_y - (int)(balance_angle_roll * RAD_TO_DEG), 
              220, horizon_y + (int)(balance_angle_roll * RAD_TO_DEG), 0x0410);  // Dark green horizon
    
    // Draw center crosshair
    draw_line(115, 120, 125, 120, 0xFFFF);
    draw_line(120, 115, 120, 125, 0xFFFF);
    
    // Draw angle indicators
    int pitch_bar = (int)(balance_angle_pitch * RAD_TO_DEG * 2);
    int roll_bar = (int)(balance_angle_roll * RAD_TO_DEG * 2);
    
    // Pitch indicator (vertical)
    if (pitch_bar > 0) {
        fill_rect(10, 120, 5, pitch_bar, 0xF800);  // Red down
    } else {
        fill_rect(10, 120 + pitch_bar, 5, -pitch_bar, 0x07E0);  // Green up
    }
    
    // Roll indicator (horizontal)
    if (roll_bar > 0) {
        fill_rect(120, 230, roll_bar, 5, 0xF800);  // Red right
    } else {
        fill_rect(120 + roll_bar, 230, -roll_bar, 5, 0x07E0);  // Green left
    }
    
    // Stability score bar
    int score_width = (int)(stability_score * 2);
    uint16_t score_color = 0x07E0;  // Green
    if (stability_score < 50) score_color = 0xF800;  // Red
    else if (stability_score < 80) score_color = 0xFFE0;  // Yellow
    
    fill_rect(20, 10, score_width, 10, score_color);
    draw_line(20, 10, 220, 10, 0x4208);  // Gray outline
    draw_line(20, 20, 220, 20, 0x4208);
    draw_line(20, 10, 20, 20, 0x4208);
    draw_line(220, 10, 220, 20, 0x4208);
}

// Draw data display
void draw_data_display(void) {
    // Draw numeric values in corners
    
    // Top left: Pitch
    int pitch_deg = (int)(balance_angle_pitch * RAD_TO_DEG);
    if (pitch_deg >= 0) {
        fill_rect(30, 30, abs(pitch_deg), 3, 0x07E0);
    } else {
        fill_rect(30 - abs(pitch_deg), 30, abs(pitch_deg), 3, 0xF800);
    }
    
    // Top right: Roll
    int roll_deg = (int)(balance_angle_roll * RAD_TO_DEG);
    if (roll_deg >= 0) {
        fill_rect(180, 30, abs(roll_deg), 3, 0x07E0);
    } else {
        fill_rect(180 - abs(roll_deg), 30, abs(roll_deg), 3, 0xF800);
    }
    
    // Bottom: Rates
    int pitch_rate = (int)(kalman_pitch.rate * RAD_TO_DEG);
    int roll_rate = (int)(kalman_roll.rate * RAD_TO_DEG);
    
    fill_rect(60, 210, abs(pitch_rate) / 2, 3, 0x001F);
    fill_rect(150, 210, abs(roll_rate) / 2, 3, 0x001F);
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    
    printf("\n=== Kalman Filter Balance Demo ===\n");
    printf("Ultra-stable orientation tracking\n");
    printf("Suitable for balancing robots\n\n");
    
    // Initialize I2C
    i2c_init(i2c1, 400000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    
    // Initialize hardware
    if (DEV_Module_Init() != 0) {
        printf("Hardware init failed!\n");
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
    
    // Initialize Kalman filters
    kalman_init(&kalman_pitch);
    kalman_init(&kalman_roll);
    
    printf("Calibrating... hold device level\n");
    
    // Initial calibration
    float pitch_offset = 0, roll_offset = 0;
    for (int i = 0; i < 100; i++) {
        float pitch, roll, yaw;
        read_imu_filtered(&pitch, &roll, &yaw);
        if (i >= 50) {  // Skip first readings
            pitch_offset += pitch;
            roll_offset += roll;
        }
        sleep_ms(10);
    }
    pitch_offset /= 50;
    roll_offset /= 50;
    
    // Apply calibration
    kalman_pitch.angle -= pitch_offset;
    kalman_roll.angle -= roll_offset;
    
    printf("Calibration complete. Running...\n");
    printf("Stability: Green=Stable, Yellow=Moderate, Red=Unstable\n");
    
    while (1) {
        // Read IMU with Kalman filtering
        float pitch, roll, yaw_angle;
        read_imu_filtered(&pitch, &roll, &yaw_angle);
        
        // Clear buffer
        clear_buffer(0x0000);
        
        // Copy and rotate cube vertices
        vertex_t rotated[8];
        for (int i = 0; i < 8; i++) {
            rotated[i] = cube[i];
            rotate_xyz(&rotated[i], pitch, roll, yaw_angle);
        }
        
        // Draw visualizations
        draw_balance_indicator();
        draw_cube(rotated);
        draw_data_display();
        
        // Display
        display_buffer();
        
        // Print debug info
        printf("\rPitch:%+6.2f° Roll:%+6.2f° | Rates: P:%+6.1f°/s R:%+6.1f°/s | Stability:%3.0f%%", 
               pitch * RAD_TO_DEG, roll * RAD_TO_DEG,
               kalman_pitch.rate * RAD_TO_DEG, kalman_roll.rate * RAD_TO_DEG,
               stability_score);
        
        // High update rate for stability
        sleep_ms(5);  // 200Hz update rate
    }
    
    return 0;
}