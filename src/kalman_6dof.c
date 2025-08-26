/*
 * 6-DOF Quaternion-based Kalman Filter
 * Stable implementation with proper noise handling and bias compensation
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
#define RAD_TO_DEG (180.0f / PI)

// Quaternion structure
typedef struct {
    float w, x, y, z;
} quaternion_t;

// Kalman filter state - simplified for stability
typedef struct {
    quaternion_t q;          // Orientation quaternion
    float gx_bias;          // Gyro X bias
    float gy_bias;          // Gyro Y bias  
    float gz_bias;          // Gyro Z bias
    float P[2][2];          // Simplified covariance for angle estimates
    float Q_angle;          // Process noise for angle
    float Q_bias;           // Process noise for bias
    float R_measure;        // Measurement noise
} kalman_state_t;

// Frame buffer
static uint16_t frame_buffer[WIDTH * HEIGHT];
static kalman_state_t kalman;
static int display_mode = 0;
static float comp_pitch = 0, comp_roll = 0, comp_yaw = 0;  // For comparison
static bool debug_stream = false;  // Stream sensor data for debugging
static int debug_counter = 0;  // Decimation counter for debug output

// Global tuning parameters for monitor control
static float Kp_gain = 2.0f;      // Proportional gain (back to normal with correct scaling)
static float Ki_gain = 0.0f;      // Integral gain disabled - bias is fixed at calibration
static float gyro_filter_alpha = 0.98f;  // Low-pass filter strength (moderate filtering)
// REMOVED gyro_scale - must use correct QMI8658 scale factor!

// Vertex structure
typedef struct {
    float x, y, z;
} vertex_t;

// Normalize quaternion
void quat_normalize(quaternion_t *q) {
    float norm = sqrtf(q->w*q->w + q->x*q->x + q->y*q->y + q->z*q->z);
    if (norm > 0.0001f) {
        float inv_norm = 1.0f / norm;
        q->w *= inv_norm;
        q->x *= inv_norm;
        q->y *= inv_norm;
        q->z *= inv_norm;
    } else {
        // Reset to identity if numerical issues
        q->w = 1.0f;
        q->x = 0.0f;
        q->y = 0.0f;
        q->z = 0.0f;
    }
}

// Initialize Kalman filter
void kalman_init(kalman_state_t *k) {
    // Identity quaternion (no rotation)
    k->q.w = 1.0f;
    k->q.x = 0.0f;
    k->q.y = 0.0f;
    k->q.z = 0.0f;
    
    // Zero initial bias
    k->gx_bias = 0.0f;
    k->gy_bias = 0.0f;
    k->gz_bias = 0.0f;
    
    // Conservative covariance initialization
    k->P[0][0] = 1.0f;
    k->P[0][1] = 0.0f;
    k->P[1][0] = 0.0f;
    k->P[1][1] = 1.0f;
    
    // Tuned noise parameters
    k->Q_angle = 0.001f;    // Low process noise - trust the model
    k->Q_bias = 0.003f;     // Bias can drift slowly
    k->R_measure = 0.03f;   // Moderate measurement noise
}

// Update quaternion from gyroscope (prediction step)
void quat_integrate(quaternion_t *q, float gx, float gy, float gz, float dt) {
    // Quaternion derivative
    float qw = q->w;
    float qx = q->x;
    float qy = q->y;
    float qz = q->z;
    
    // Integration with small angle approximation for stability
    float half_dt = 0.5f * dt;
    q->w += half_dt * (-qx*gx - qy*gy - qz*gz);
    q->x += half_dt * ( qw*gx + qy*gz - qz*gy);
    q->y += half_dt * ( qw*gy - qx*gz + qz*gx);
    q->z += half_dt * ( qw*gz + qx*gy - qy*gx);
    
    // Normalize to prevent drift
    quat_normalize(q);
}

// Get gravity vector from quaternion
void quat_to_gravity(const quaternion_t *q, float *gx, float *gy, float *gz) {
    // Gravity in body frame when quaternion represents rotation from world to body
    *gx = 2.0f * (q->x*q->z - q->w*q->y);
    *gy = 2.0f * (q->w*q->x + q->y*q->z);
    *gz = q->w*q->w - q->x*q->x - q->y*q->y + q->z*q->z;
}

// Simplified Kalman update using accelerometer
void kalman_update_accel(kalman_state_t *k, float ax, float ay, float az, float dt) {
    // Normalize accelerometer
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm < 0.5f || norm > 2.0f) return;  // Reject bad readings
    
    ax /= norm;
    ay /= norm;
    az /= norm;
    
    // Get expected gravity from current quaternion
    float gx, gy, gz;
    quat_to_gravity(&k->q, &gx, &gy, &gz);
    
    // Error between measured and expected gravity
    float ex = ay*gz - az*gy;
    float ey = az*gx - ax*gz;
    float ez = ax*gy - ay*gx;
    
    // Adaptive gain based on acceleration magnitude
    float accel_confidence = 1.0f - fabsf(norm - 1.0f);
    if (accel_confidence < 0.0f) accel_confidence = 0.0f;
    
    // Apply PI controller for error correction (using global tunable parameters)
    float Kp = Kp_gain * accel_confidence;  // Use global Kp_gain
    // Ki disabled - bias should only be set during calibration
    // float Ki = Ki_gain * accel_confidence;
    
    // Update bias estimate (integral term) - DISABLED to prevent drift
    // The bias should only be set during calibration when stationary
    // k->gx_bias += Ki * ex * dt;
    // k->gy_bias += Ki * ey * dt;
    // k->gz_bias += Ki * ez * dt;
    
    // Limit bias
    float max_bias = 0.2f;  // rad/s
    if (k->gx_bias >  max_bias) k->gx_bias =  max_bias;
    if (k->gx_bias < -max_bias) k->gx_bias = -max_bias;
    if (k->gy_bias >  max_bias) k->gy_bias =  max_bias;
    if (k->gy_bias < -max_bias) k->gy_bias = -max_bias;
    if (k->gz_bias >  max_bias) k->gz_bias =  max_bias;
    if (k->gz_bias < -max_bias) k->gz_bias = -max_bias;
    
    // Apply proportional correction to quaternion
    float correction_x = Kp * ex;
    float correction_y = Kp * ey;
    float correction_z = Kp * ez;
    
    // Apply correction as a small rotation
    k->q.w -= dt * 0.5f * (k->q.x * correction_x + k->q.y * correction_y + k->q.z * correction_z);
    k->q.x += dt * 0.5f * (k->q.w * correction_x + k->q.y * correction_z - k->q.z * correction_y);
    k->q.y += dt * 0.5f * (k->q.w * correction_y - k->q.x * correction_z + k->q.z * correction_x);
    k->q.z += dt * 0.5f * (k->q.w * correction_z + k->q.x * correction_y - k->q.y * correction_x);
    
    quat_normalize(&k->q);
}

// Convert quaternion to Euler angles for display
void quat_to_euler(const quaternion_t *q, float *pitch, float *roll, float *yaw) {
    // Roll (x-axis rotation)
    float sinr_cosp = 2.0f * (q->w * q->x + q->y * q->z);
    float cosr_cosp = 1.0f - 2.0f * (q->x * q->x + q->y * q->y);
    *roll = atan2f(sinr_cosp, cosr_cosp);
    
    // Pitch (y-axis rotation)
    float sinp = 2.0f * (q->w * q->y - q->z * q->x);
    if (fabsf(sinp) >= 1.0f)
        *pitch = copysignf(PI / 2.0f, sinp);
    else
        *pitch = asinf(sinp);
    
    // Yaw (z-axis rotation)
    float siny_cosp = 2.0f * (q->w * q->z + q->x * q->y);
    float cosy_cosp = 1.0f - 2.0f * (q->y * q->y + q->z * q->z);
    *yaw = atan2f(siny_cosp, cosy_cosp);
}

// Rotate vertex using quaternion
void rotate_vertex_quat(vertex_t *v, const quaternion_t *q) {
    float qw = q->w, qx = q->x, qy = q->y, qz = q->z;
    float vx = v->x, vy = v->y, vz = v->z;
    
    // Optimized quaternion rotation (avoiding full multiplication)
    float tx = 2.0f * (qy*vz - qz*vy);
    float ty = 2.0f * (qz*vx - qx*vz);
    float tz = 2.0f * (qx*vy - qy*vx);
    
    v->x = vx + qw*tx + qy*tz - qz*ty;
    v->y = vy + qw*ty + qz*tx - qx*tz;
    v->z = vz + qw*tz + qx*ty - qy*tx;
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
    int edges[12][2] = {
        {0,1}, {1,2}, {2,3}, {3,0},
        {4,5}, {5,6}, {6,7}, {7,4},
        {0,4}, {1,5}, {2,6}, {3,7}
    };
    
    for (int i = 0; i < 12; i++) {
        const vertex_t *v0 = &vertices[edges[i][0]];
        const vertex_t *v1 = &vertices[edges[i][1]];
        
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

// Draw angle indicator
void draw_angle_bar(int x, int y, float angle, uint16_t color) {
    int width = 80;
    int pos = x + (int)(angle * RAD_TO_DEG * width / 180.0f);
    
    draw_line(x - width/2, y, x + width/2, y, 0x4208);
    
    if (pos >= x - width/2 && pos <= x + width/2) {
        draw_line(pos - 2, y - 3, pos - 2, y + 3, color);
        draw_line(pos - 1, y - 3, pos - 1, y + 3, color);
        draw_line(pos, y - 3, pos, y + 3, color);
        draw_line(pos + 1, y - 3, pos + 1, y + 3, color);
        draw_line(pos + 2, y - 3, pos + 2, y + 3, color);
    }
    
    draw_line(x, y - 5, x, y + 5, 0xFFFF);
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
    // CTRL1: Enable acc and gyro with proper range
    // Bit 7-6: reserved
    // Bit 5-4: acc range (00=2g)
    // Bit 3-2: gyro range (01=256dps for 128 LSB/dps)
    // Bit 1: gyro enable
    // Bit 0: acc enable
    i2c_write_register(QMI8658_ADDR, QMI8658_CTRL1, 0x47); // 0100 0111 = 2g acc, 256dps gyro, both enabled
    
    // CTRL2: Accelerometer ODR = 250Hz (0x05)
    i2c_write_register(QMI8658_ADDR, QMI8658_CTRL2, 0x05);
    
    // CTRL3: Gyroscope ODR = 250Hz (0x50)
    i2c_write_register(QMI8658_ADDR, QMI8658_CTRL3, 0x50);
    
    // CTRL7: Enable sensors
    i2c_write_register(QMI8658_ADDR, QMI8658_CTRL7, 0x03);
    
    return true;
}

// Process command - enhanced with parameter commands
void process_command(char c) {
    static bool parse_mode = false;
    static char parse_buffer[32];
    static int parse_index = 0;
    
    // Check for parameter setting commands (format: P<param>=<value>)
    if (c == 'P') {
        parse_mode = true;
        parse_index = 0;
        return;
    }
    
    if (parse_mode) {
        if (c == '\n' || c == '\r') {
            parse_buffer[parse_index] = '\0';
            // Parse parameter command
            char param;
            float value;
            if (sscanf(parse_buffer, "%c=%f", &param, &value) == 2) {
                switch(param) {
                    case 'K': // Kp gain
                        Kp_gain = value;
                        printf("Set Kp_gain=%.3f\n", Kp_gain);
                        break;
                    case 'I': // Ki gain
                        Ki_gain = value;
                        printf("Set Ki_gain=%.5f\n", Ki_gain);
                        break;
                    case 'F': // Filter alpha
                        gyro_filter_alpha = value;
                        printf("Set filter_alpha=%.3f\n", gyro_filter_alpha);
                        break;
                    case 'S': // Deprecated - scale is fixed by QMI8658
                        printf("Scale is fixed at 128 LSB/dps for QMI8658\n");
                        break;
                    case 'B': { // Direct bias set (x,y,z in one command)
                        // Format: PB=0.01,0.02,0.03
                        float bx, by, bz;
                        if (sscanf(parse_buffer, "B=%f,%f,%f", &bx, &by, &bz) == 3) {
                            kalman.gx_bias = bx;
                            kalman.gy_bias = by;
                            kalman.gz_bias = bz;
                            printf("Set bias=[%.4f,%.4f,%.4f]\n", bx, by, bz);
                        }
                        break;
                    }
                }
            }
            parse_mode = false;
        } else if (parse_index < 31) {
            parse_buffer[parse_index++] = c;
        }
        return;
    }
    
    // Normal single-character commands
    switch(c) {
        case 'r': {
            // Save current bias before reset
            float saved_gx_bias = kalman.gx_bias;
            float saved_gy_bias = kalman.gy_bias;
            float saved_gz_bias = kalman.gz_bias;
            
            // Reset quaternion but keep other parameters
            kalman.q.w = 1.0f;
            kalman.q.x = 0.0f;
            kalman.q.y = 0.0f;
            kalman.q.z = 0.0f;
            
            // Restore bias values - CRITICAL!
            kalman.gx_bias = saved_gx_bias;
            kalman.gy_bias = saved_gy_bias;
            kalman.gz_bias = saved_gz_bias;
            
            // Reset complementary filter angles
            comp_pitch = 0;
            comp_roll = 0;
            comp_yaw = 0;
            
            printf("Reset orientation (bias preserved: %.4f,%.4f,%.4f)\n", 
                   kalman.gx_bias, kalman.gy_bias, kalman.gz_bias);
            break;
        }
        case 'm':
            display_mode = (display_mode + 1) % 3;
            printf("Mode: %d (0=Kalman, 1=Comp, 2=Both)\n", display_mode);
            break;
        case '+':
            kalman.R_measure += 0.01f;
            printf("R_measure: %.3f (trust accel less)\n", kalman.R_measure);
            break;
        case '-':
            kalman.R_measure -= 0.01f;
            if (kalman.R_measure < 0.001f) kalman.R_measure = 0.001f;
            printf("R_measure: %.3f (trust accel more)\n", kalman.R_measure);
            break;
        case 'q':
            kalman.Q_angle += 0.001f;
            printf("Q_angle: %.4f\n", kalman.Q_angle);
            break;
        case 'a':
            kalman.Q_angle -= 0.001f;
            if (kalman.Q_angle < 0.0001f) kalman.Q_angle = 0.0001f;
            printf("Q_angle: %.4f\n", kalman.Q_angle);
            break;
        case 'd':
            debug_stream = !debug_stream;
            if (debug_stream) {
                printf("DEBUG_START\n");  // Marker for parser
                printf("# CSV format: time_ms,raw_gx,raw_gy,raw_gz,raw_ax,raw_ay,raw_az,");
                printf("filt_gx,filt_gy,filt_gz,bias_x,bias_y,bias_z,");
                printf("qw,qx,qy,qz,pitch,roll,yaw,Kp,Ki,filter,scale\n");
            } else {
                printf("DEBUG_STOP\n");  // Marker for parser
            }
            break;
        case 'p': // Print current parameters
            printf("PARAMS: Kp=%.3f Filter=%.3f ", 
                   Kp_gain, gyro_filter_alpha);
            printf("Bias=[%.4f,%.4f,%.4f] ", kalman.gx_bias, kalman.gy_bias, kalman.gz_bias);
            printf("Q_angle=%.4f Q_bias=%.4f R=%.3f\n", 
                   kalman.Q_angle, kalman.Q_bias, kalman.R_measure);
            break;
        case 'h':
            printf("\n=== 6-DOF Kalman Controls ===\n");
            printf("r: Reset orientation\n");
            printf("m: Switch display mode\n");
            printf("d: Toggle debug streaming\n");
            printf("p: Print current parameters\n");
            printf("+/-: Adjust R_measure (accel trust)\n");
            printf("q/a: Adjust Q_angle (process noise)\n");
            printf("P<X>=<val>: Set parameter (K,I,F,S,B)\n");
            printf("==============================\n");
            break;
    }
}

int main() {
    stdio_init_all();
    sleep_ms(2000);
    
    printf("6-DOF Quaternion Kalman Filter\n");
    
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
    
    // Initialize Kalman filter
    kalman_init(&kalman);
    
    // Calibrate gyro bias
    printf("Calibrating gyro (keep still)...\n");
    float cal_gx = 0, cal_gy = 0, cal_gz = 0;
    for (int i = 0; i < 200; i++) {
        int16_t raw_gx = i2c_read_16bit(QMI8658_ADDR, QMI8658_GX_L);
        int16_t raw_gy = i2c_read_16bit(QMI8658_ADDR, QMI8658_GX_L + 2);
        int16_t raw_gz = i2c_read_16bit(QMI8658_ADDR, QMI8658_GX_L + 4);
        cal_gx += raw_gx / 128.0f;
        cal_gy += raw_gy / 128.0f;
        cal_gz += raw_gz / 128.0f;
        sleep_ms(5);
    }
    
    // Start with zero bias - the automatic calibration is causing drift
    // User can apply calibration with PB command after measuring actual bias
    kalman.gx_bias = 0.0f;
    kalman.gy_bias = 0.0f;
    kalman.gz_bias = 0.0f;
    
    printf("Note: Starting with zero bias. Use PB command to calibrate if needed");
    
    printf("Calibration done. Running...\n");
    printf("Press 'h' for help\n");
    
    // Define cube
    vertex_t cube[8] = {
        {-0.5f, -0.5f, -0.5f}, {0.5f, -0.5f, -0.5f},
        {0.5f, 0.5f, -0.5f}, {-0.5f, 0.5f, -0.5f},
        {-0.5f, -0.5f, 0.5f}, {0.5f, -0.5f, 0.5f},
        {0.5f, 0.5f, 0.5f}, {-0.5f, 0.5f, 0.5f}
    };
    
    uint32_t last_time = time_us_32();
    const float alpha = 0.98f;  // Complementary filter constant
    
    // Low-pass filter state for smoothing
    float filtered_gx = 0, filtered_gy = 0, filtered_gz = 0;
    
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
        
        // Convert to physical units using CORRECT QMI8658 scale factors
        // Accelerometer: ±2g range = 16384 LSB/g
        float ax = raw_ax / 16384.0f;
        float ay = raw_ay / 16384.0f;
        float az = raw_az / 16384.0f;
        
        // Gyroscope: ±256 dps range = 128 LSB/dps (from QMI8658 datasheet)
        // With axis swapping for correct orientation
        float gx = (raw_gy / 128.0f) * DEG_TO_RAD;   // Swap X/Y
        float gy = -(raw_gx / 128.0f) * DEG_TO_RAD;  // Swap and invert
        float gz = -(raw_gz / 128.0f) * DEG_TO_RAD;  // Invert Z
        
        // Calculate dt
        uint32_t now = time_us_32();
        float dt = (now - last_time) / 1000000.0f;
        if (dt > 0.1f) dt = 0.01f;  // Limit for first iteration
        if (dt < 0.001f) dt = 0.001f;  // Minimum dt
        last_time = now;
        
        // Apply low-pass filter to gyro for smoothness (using tunable alpha)
        filtered_gx = gyro_filter_alpha * filtered_gx + (1.0f - gyro_filter_alpha) * gx;
        filtered_gy = gyro_filter_alpha * filtered_gy + (1.0f - gyro_filter_alpha) * gy;
        filtered_gz = gyro_filter_alpha * filtered_gz + (1.0f - gyro_filter_alpha) * gz;
        
        // Apply Kalman filter
        // Predict step: integrate filtered gyroscope
        float corrected_gx = filtered_gx - kalman.gx_bias;
        float corrected_gy = filtered_gy - kalman.gy_bias;
        float corrected_gz = filtered_gz - kalman.gz_bias;
        
        quat_integrate(&kalman.q, corrected_gx, corrected_gy, corrected_gz, dt);
        
        // Update step: correct with accelerometer
        kalman_update_accel(&kalman, ax, ay, az, dt);
        
        // Get Euler angles for display
        float kalman_pitch, kalman_roll, kalman_yaw;
        quat_to_euler(&kalman.q, &kalman_pitch, &kalman_roll, &kalman_yaw);
        
        // Complementary filter for comparison (using same filtered values)
        float accel_pitch = atan2f(ax, sqrtf(ay*ay + az*az));  // Fixed for proper axes
        float accel_roll = atan2f(ay, sqrtf(ax*ax + az*az));
        
        comp_pitch = alpha * (comp_pitch + corrected_gx * dt) + (1.0f - alpha) * accel_pitch;
        comp_roll = alpha * (comp_roll + corrected_gy * dt) + (1.0f - alpha) * accel_roll;
        comp_yaw += corrected_gz * dt;
        
        // Wrap angles
        if (comp_yaw > PI) comp_yaw -= TWO_PI;
        if (comp_yaw < -PI) comp_yaw += TWO_PI;
        
        // Clear display
        clear_buffer(0x0000);
        
        // Draw based on mode
        if (display_mode == 0 || display_mode == 2) {
            // Kalman cube
            vertex_t kalman_cube[8];
            for (int i = 0; i < 8; i++) {
                kalman_cube[i] = cube[i];
                rotate_vertex_quat(&kalman_cube[i], &kalman.q);
            }
            draw_cube(kalman_cube, 0x07E0);  // Green
            
            // Show angles
            draw_angle_bar(120, 200, kalman_pitch, 0x07E0);
            draw_angle_bar(120, 210, kalman_roll, 0x07E0);
        }
        
        if (display_mode == 1 || display_mode == 2) {
            // Complementary filter cube
            vertex_t comp_cube[8];
            for (int i = 0; i < 8; i++) {
                comp_cube[i] = cube[i];
                if (display_mode == 2) {
                    // Scale down for comparison
                    comp_cube[i].x *= 0.5f;
                    comp_cube[i].y *= 0.5f;
                    comp_cube[i].z *= 0.5f;
                }
                
                // Rotate using Euler angles
                float cx = comp_cube[i].x;
                float cy = comp_cube[i].y;
                float cz = comp_cube[i].z;
                
                // Rotate around X (pitch)
                float y1 = cy * cosf(comp_pitch) - cz * sinf(comp_pitch);
                float z1 = cy * sinf(comp_pitch) + cz * cosf(comp_pitch);
                
                // Rotate around Y (roll)
                float x2 = cx * cosf(comp_roll) + z1 * sinf(comp_roll);
                float z2 = -cx * sinf(comp_roll) + z1 * cosf(comp_roll);
                
                // Rotate around Z (yaw)
                comp_cube[i].x = x2 * cosf(comp_yaw) - y1 * sinf(comp_yaw);
                comp_cube[i].y = x2 * sinf(comp_yaw) + y1 * cosf(comp_yaw);
                comp_cube[i].z = z2;
            }
            draw_cube(comp_cube, 0xF800);  // Red
            
            if (display_mode == 1) {
                draw_angle_bar(120, 200, comp_pitch, 0xF800);
                draw_angle_bar(120, 210, comp_roll, 0xF800);
            }
        }
        
        // Update display
        LCD_1IN28_Display(frame_buffer);
        
        // Debug streaming output (decimated to ~20Hz)
        if (debug_stream) {
            debug_counter++;
            if (debug_counter >= 5) {  // Output every 5 frames (~20Hz at 100Hz loop)
                debug_counter = 0;
                uint32_t time_ms = time_us_32() / 1000;
                
                // Output CSV line with all sensor data
                printf("%lu,", time_ms);
                // Raw gyro (before any processing)
                printf("%.3f,%.3f,%.3f,", gx*RAD_TO_DEG, gy*RAD_TO_DEG, gz*RAD_TO_DEG);
                // Raw accel
                printf("%.3f,%.3f,%.3f,", ax, ay, az);
                // Filtered gyro
                printf("%.3f,%.3f,%.3f,", filtered_gx*RAD_TO_DEG, filtered_gy*RAD_TO_DEG, filtered_gz*RAD_TO_DEG);
                // Current bias estimates
                printf("%.4f,%.4f,%.4f,", kalman.gx_bias, kalman.gy_bias, kalman.gz_bias);
                // Quaternion
                printf("%.4f,%.4f,%.4f,%.4f,", kalman.q.w, kalman.q.x, kalman.q.y, kalman.q.z);
                // Euler angles
                printf("%.2f,%.2f,%.2f,", kalman_pitch*RAD_TO_DEG, kalman_roll*RAD_TO_DEG, kalman_yaw*RAD_TO_DEG);
                // Current tuning parameters (removed scale - it's fixed at 128)
                printf("%.3f,%.5f,%.3f\n", Kp_gain, Ki_gain, gyro_filter_alpha);
            }
        }
        
        // Small delay for stable loop timing
        sleep_ms(5);
    }
    
    return 0;
}