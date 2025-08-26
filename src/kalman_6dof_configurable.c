// Enhanced 6-DOF Kalman filter with full axis configuration
// Based on kalman_6dof.c with added axis remapping and testing

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "hardware/timer.h"
#include "hardware/gpio.h"
#include "LCD_1in28.h"
#include "DEV_Config.h"

// Display and sensor defines (same as before)
#define LCD_SPI_PORT spi1
#define LCD_WIDTH 240
#define LCD_HEIGHT 240
#define LCD_CS_PIN 9
#define LCD_DC_PIN 8
#define LCD_RST_PIN 12
#define LCD_BL_PIN 25
#define LCD_SPI_CLK_PIN 10
#define LCD_SPI_MOSI_PIN 11

#define IMU_I2C_PORT i2c1
#define IMU_SDA_PIN 6
#define IMU_SCL_PIN 7
#define QMI8658_ADDR 0x6B
#define QMI8658_CTRL1 0x02
#define QMI8658_CTRL2 0x03
#define QMI8658_CTRL3 0x04
#define QMI8658_CTRL7 0x08
#define QMI8658_DATA 0x35

#define DEG_TO_RAD (M_PI / 180.0f)
#define RAD_TO_DEG (180.0f / M_PI)

// Color definitions (RGB565)
#define BLACK   0x0000
#define WHITE   0xFFFF
#define RED     0xF800
#define GREEN   0x07E0
#define BLUE    0x001F

// Axis configuration structure
typedef struct {
    // Axis mapping (0=X, 1=Y, 2=Z)
    int gyro_map[3];    // Maps physical axes to logical axes
    int accel_map[3];
    
    // Axis polarity (-1 or 1)
    float gyro_sign[3];
    float accel_sign[3];
    
    // Axis scaling/magnitude adjustment
    float gyro_scale[3];
    float accel_scale[3];
    
    // Response tuning per axis
    float gyro_response[3];  // Filter/smoothing per axis
    float accel_weight[3];   // Trust weight per axis
} AxisConfig;

// Global axis configuration
static AxisConfig axis_config = {
    .gyro_map = {1, 0, 2},      // Default: swap X/Y
    .accel_map = {1, 0, 2},
    .gyro_sign = {1.0f, -1.0f, -1.0f},  // Default: invert Y and Z
    .accel_sign = {1.0f, -1.0f, 1.0f},
    .gyro_scale = {1.0f, 1.0f, 1.0f},
    .accel_scale = {1.0f, 1.0f, 1.0f},
    .gyro_response = {0.98f, 0.98f, 0.98f},
    .accel_weight = {1.0f, 1.0f, 1.0f}
};

// Test mode for systematic testing
typedef enum {
    TEST_OFF = 0,
    TEST_GYRO_X,
    TEST_GYRO_Y,
    TEST_GYRO_Z,
    TEST_ACCEL_X,
    TEST_ACCEL_Y,
    TEST_ACCEL_Z,
    TEST_COMBINED
} TestMode;

static TestMode test_mode = TEST_OFF;
static float test_amplitude = 0.0f;
static uint32_t test_start_time = 0;

// Vertex structure for 3D rendering
typedef struct {
    float x, y, z;
} Vertex;

// Quaternion and Kalman structures (same as before)
typedef struct {
    float w, x, y, z;
} Quaternion;

typedef struct {
    Quaternion q;
    float gx_bias, gy_bias, gz_bias;
    float P[4][4];
    float Q_angle, Q_bias;
    float R_measure;
} KalmanState;

static KalmanState kalman;
static uint16_t framebuffer[LCD_WIDTH * LCD_HEIGHT];
static bool debug_stream = false;
// static int display_mode = 0;  // Not used in simplified version

// Kalman filter parameters
static float Kp_gain = 2.0f;
static float Ki_gain = 0.001f;
static float gyro_filter_alpha = 0.98f;

// Apply axis configuration to raw sensor data
void apply_axis_config(float raw[3], float output[3], int map[3], float sign[3], float scale[3]) {
    for (int i = 0; i < 3; i++) {
        output[i] = raw[map[i]] * sign[i] * scale[i];
    }
}

// Process test mode input
void apply_test_signal(float* gx, float* gy, float* gz, float* ax, float* ay, float* az) {
    if (test_mode == TEST_OFF) return;
    
    float elapsed = (absolute_time_diff_us(test_start_time, get_absolute_time()) / 1000000.0f);
    float signal = test_amplitude * sinf(elapsed * 2.0f * M_PI * 0.5f); // 0.5 Hz test signal
    
    switch(test_mode) {
        case TEST_OFF:
            return;  // No test signal
        case TEST_GYRO_X:
            *gx += signal;
            printf("TEST: Gyro X = %.2f°/s\n", signal * RAD_TO_DEG);
            break;
        case TEST_GYRO_Y:
            *gy += signal;
            printf("TEST: Gyro Y = %.2f°/s\n", signal * RAD_TO_DEG);
            break;
        case TEST_GYRO_Z:
            *gz += signal;
            printf("TEST: Gyro Z = %.2f°/s\n", signal * RAD_TO_DEG);
            break;
        case TEST_ACCEL_X:
            *ax += signal * 0.1f; // Scale down for accel
            printf("TEST: Accel X = %.3fg\n", signal * 0.1f);
            break;
        case TEST_ACCEL_Y:
            *ay += signal * 0.1f;
            printf("TEST: Accel Y = %.3fg\n", signal * 0.1f);
            break;
        case TEST_ACCEL_Z:
            *az += signal * 0.1f;
            printf("TEST: Accel Z = %.3fg\n", signal * 0.1f);
            break;
        case TEST_COMBINED:
            *gx += signal * 0.5f;
            *gy += signal * 0.3f;
            *gz += signal * 0.2f;
            printf("TEST: Combined P:%.1f° R:%.1f° Y:%.1f°\n", 
                   signal*28.6f, signal*17.2f, signal*11.5f);
            break;
    }
}

// Forward declarations
void quat_integrate(Quaternion* q, float gx, float gy, float gz, float dt);

// Enhanced command processing
void process_command(char c) {
    static bool parse_mode = false;
    static char parse_buffer[32];
    static int parse_index = 0;
    static char param = 0;
    
    if (parse_mode) {
        if (c == '\n' || c == '\r') {
            parse_buffer[parse_index] = '\0';
            
            if (param == 'P') {
                // Parameter commands
                float value;
                if (sscanf(parse_buffer, "%c=%f", &param, &value) == 2) {
                    switch(param) {
                        case 'K': Kp_gain = value; break;
                        case 'I': Ki_gain = value; break;
                        case 'F': gyro_filter_alpha = value; break;
                        case 'B': // Bias setting
                            {
                                float bx, by, bz;
                                if (sscanf(parse_buffer, "B=%f,%f,%f", &bx, &by, &bz) == 3) {
                                    kalman.gx_bias = bx;
                                    kalman.gy_bias = by;
                                    kalman.gz_bias = bz;
                                    printf("Set bias=[%.4f,%.4f,%.4f]\n", bx, by, bz);
                                }
                            }
                            break;
                    }
                }
            } else if (param == 'A') {
                // Axis configuration commands
                char axis_cmd;
                int axis;
                float value;
                if (sscanf(parse_buffer, "%c%d=%f", &axis_cmd, &axis, &value) == 3 && axis >= 0 && axis < 3) {
                    switch(axis_cmd) {
                        case 'G': // Gyro sign
                            axis_config.gyro_sign[axis] = (value < 0) ? -1.0f : 1.0f;
                            printf("Gyro axis %d sign = %.0f\n", axis, axis_config.gyro_sign[axis]);
                            break;
                        case 'A': // Accel sign
                            axis_config.accel_sign[axis] = (value < 0) ? -1.0f : 1.0f;
                            printf("Accel axis %d sign = %.0f\n", axis, axis_config.accel_sign[axis]);
                            break;
                        case 'S': // Scale
                            axis_config.gyro_scale[axis] = value;
                            printf("Gyro axis %d scale = %.2f\n", axis, value);
                            break;
                        case 'W': // Accel weight
                            axis_config.accel_weight[axis] = value;
                            printf("Accel axis %d weight = %.2f\n", axis, value);
                            break;
                    }
                } else if (sscanf(parse_buffer, "MAP=%d,%d,%d", 
                                  &axis_config.gyro_map[0], 
                                  &axis_config.gyro_map[1], 
                                  &axis_config.gyro_map[2]) == 3) {
                    printf("Gyro map: X->%d Y->%d Z->%d\n", 
                           axis_config.gyro_map[0], axis_config.gyro_map[1], axis_config.gyro_map[2]);
                }
            } else if (param == 'T') {
                // Test mode commands
                char test_cmd;
                float amp;
                if (sscanf(parse_buffer, "%c=%f", &test_cmd, &amp) == 2) {
                    test_amplitude = amp * DEG_TO_RAD;
                    test_start_time = get_absolute_time();
                    switch(test_cmd) {
                        case 'X': test_mode = TEST_GYRO_X; printf("Testing Gyro X, amplitude %.1f°\n", amp); break;
                        case 'Y': test_mode = TEST_GYRO_Y; printf("Testing Gyro Y, amplitude %.1f°\n", amp); break;
                        case 'Z': test_mode = TEST_GYRO_Z; printf("Testing Gyro Z, amplitude %.1f°\n", amp); break;
                        case 'A': test_mode = TEST_ACCEL_X; printf("Testing Accel X\n"); break;
                        case 'B': test_mode = TEST_ACCEL_Y; printf("Testing Accel Y\n"); break;
                        case 'C': test_mode = TEST_ACCEL_Z; printf("Testing Accel Z\n"); break;
                        case 'M': test_mode = TEST_COMBINED; printf("Testing combined motion\n"); break;
                        case '0': test_mode = TEST_OFF; printf("Test mode OFF\n"); break;
                    }
                }
            } else if (param == 'I') {
                // IMU data injection for regression testing
                // Format: I<ax>,<ay>,<az>,<gx>,<gy>,<gz>
                float ax, ay, az, gx, gy, gz;
                if (sscanf(parse_buffer, "%f,%f,%f,%f,%f,%f", &ax, &ay, &az, &gx, &gy, &gz) == 6) {
                    // Inject test data directly into Kalman filter
                    // Convert gyro from deg/s to rad/s
                    float test_gx = gx * DEG_TO_RAD;
                    float test_gy = gy * DEG_TO_RAD;
                    float test_gz = gz * DEG_TO_RAD;
                    
                    // Get time delta
                    static uint32_t last_inject_time = 0;
                    uint32_t current_time = time_us_32();
                    float dt = 0.02f; // Default 50Hz
                    if (last_inject_time != 0) {
                        dt = (current_time - last_inject_time) / 1000000.0f;
                    }
                    last_inject_time = current_time;
                    
                    // Apply directly to Kalman filter
                    float corrected_gx = test_gx - kalman.gx_bias;
                    float corrected_gy = test_gy - kalman.gy_bias;
                    float corrected_gz = test_gz - kalman.gz_bias;
                    
                    quat_integrate(&kalman.q, corrected_gx, corrected_gy, corrected_gz, dt);
                    
                    // Get Euler angles for output
                    float pitch = atan2f(2.0f*(kalman.q.w*kalman.q.x + kalman.q.y*kalman.q.z),
                                        1.0f - 2.0f*(kalman.q.x*kalman.q.x + kalman.q.y*kalman.q.y));
                    float roll = asinf(2.0f*(kalman.q.w*kalman.q.y - kalman.q.z*kalman.q.x));
                    float yaw = atan2f(2.0f*(kalman.q.w*kalman.q.z + kalman.q.x*kalman.q.y),
                                      1.0f - 2.0f*(kalman.q.y*kalman.q.y + kalman.q.z*kalman.q.z));
                    
                    // Output CSV format for analysis
                    printf("CSV,%lu,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\n",
                           current_time/1000,
                           ax, ay, az,
                           gx, gy, gz,
                           pitch*RAD_TO_DEG, roll*RAD_TO_DEG, yaw*RAD_TO_DEG);
                }
            }
            parse_mode = false;
            parse_index = 0;
        } else if (parse_index < 31) {
            parse_buffer[parse_index++] = c;
        }
    } else if (c == 'P' || c == 'A' || c == 'T' || c == 'I') {
        parse_mode = true;
        param = c;
        parse_index = 0;
    } else {
        // Single character commands
        switch(c) {
            case 'r': // Reset orientation
                kalman.q.w = 1.0f;
                kalman.q.x = 0.0f;
                kalman.q.y = 0.0f;
                kalman.q.z = 0.0f;
                printf("Reset orientation\n");
                break;
                
            case 'd': // Toggle debug stream
                debug_stream = !debug_stream;
                if (debug_stream) {
                    printf("Debug stream ON - CSV format\n");
                } else {
                    printf("Debug stream OFF\n");
                }
                break;
                
            case 'p': // Print configuration
                printf("AXIS CONFIG:\n");
                printf("  Gyro map: [%d,%d,%d] sign: [%.0f,%.0f,%.0f] scale: [%.2f,%.2f,%.2f]\n",
                       axis_config.gyro_map[0], axis_config.gyro_map[1], axis_config.gyro_map[2],
                       axis_config.gyro_sign[0], axis_config.gyro_sign[1], axis_config.gyro_sign[2],
                       axis_config.gyro_scale[0], axis_config.gyro_scale[1], axis_config.gyro_scale[2]);
                printf("  Accel map: [%d,%d,%d] sign: [%.0f,%.0f,%.0f]\n",
                       axis_config.accel_map[0], axis_config.accel_map[1], axis_config.accel_map[2],
                       axis_config.accel_sign[0], axis_config.accel_sign[1], axis_config.accel_sign[2]);
                printf("  Test mode: %d, amplitude: %.1f°\n", test_mode, test_amplitude * RAD_TO_DEG);
                break;
                
            case 'h': // Help
                printf("\n=== AXIS CONFIGURATION HELP ===\n");
                printf("Axis commands:\n");
                printf("  AG0=-1  : Flip gyro X axis polarity\n");
                printf("  AG1=1   : Set gyro Y axis normal polarity\n");
                printf("  AA2=-1  : Flip accel Z axis polarity\n");
                printf("  AS0=1.5 : Scale gyro X by 1.5x\n");
                printf("  AW1=0.5 : Reduce accel Y weight to 0.5\n");
                printf("  AMAP=1,0,2 : Remap axes (swap X/Y)\n");
                printf("\nTest commands:\n");
                printf("  TX=30   : Test gyro X with 30° amplitude\n");
                printf("  TY=20   : Test gyro Y with 20° amplitude\n");
                printf("  TZ=10   : Test gyro Z with 10° amplitude\n");
                printf("  TA=1    : Test accel X\n");
                printf("  TM=45   : Test combined motion\n");
                printf("  T0=0    : Stop testing\n");
                printf("\nRegression Testing:\n");
                printf("  I<ax>,<ay>,<az>,<gx>,<gy>,<gz> : Inject IMU data\n");
                printf("    Example: I0.0,0.0,1.0,30.0,0.0,0.0\n");
                printf("\nOther: r=reset, d=debug, p=print config\n");
                break;
        }
    }
}

// I2C functions
void i2c_write_register(uint8_t addr, uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    i2c_write_blocking(IMU_I2C_PORT, addr, data, 2, false);
}

int16_t i2c_read_register_int16(uint8_t addr, uint8_t reg) {
    uint8_t buf[2];
    i2c_write_blocking(IMU_I2C_PORT, addr, &reg, 1, true);
    i2c_read_blocking(IMU_I2C_PORT, addr, buf, 2, false);
    return (int16_t)(buf[1] << 8 | buf[0]);
}

// QMI8658 init with proper configuration
bool qmi8658_init(void) {
    i2c_write_register(QMI8658_ADDR, QMI8658_CTRL1, 0x47); // 256dps gyro, 2g accel
    i2c_write_register(QMI8658_ADDR, QMI8658_CTRL2, 0x05); // 250Hz accel
    i2c_write_register(QMI8658_ADDR, QMI8658_CTRL3, 0x50); // 250Hz gyro
    i2c_write_register(QMI8658_ADDR, QMI8658_CTRL7, 0x03); // Enable
    return true;
}

// Quaternion operations
void quat_normalize(Quaternion* q) {
    float norm = sqrtf(q->w*q->w + q->x*q->x + q->y*q->y + q->z*q->z);
    if (norm > 0.0f) {
        q->w /= norm;
        q->x /= norm;
        q->y /= norm;
        q->z /= norm;
    }
}

void quat_integrate(Quaternion* q, float gx, float gy, float gz, float dt) {
    float qw = q->w, qx = q->x, qy = q->y, qz = q->z;
    
    q->w += 0.5f * (-qx*gx - qy*gy - qz*gz) * dt;
    q->x += 0.5f * (qw*gx + qy*gz - qz*gy) * dt;
    q->y += 0.5f * (qw*gy - qx*gz + qz*gx) * dt;
    q->z += 0.5f * (qw*gz + qx*gy - qy*gx) * dt;
    
    quat_normalize(q);
}

// Kalman filter init
void kalman_init(KalmanState* k) {
    k->q.w = 1.0f;
    k->q.x = 0.0f;
    k->q.y = 0.0f;
    k->q.z = 0.0f;
    
    k->gx_bias = 0.0f;
    k->gy_bias = 0.0f;
    k->gz_bias = 0.0f;
    
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) {
            k->P[i][j] = (i == j) ? 0.1f : 0.0f;
        }
    }
    
    k->Q_angle = 0.001f;
    k->Q_bias = 0.003f;
    k->R_measure = 0.03f;
}

// LCD functions using Waveshare library
void lcd_init(void) {
    DEV_Module_Init();
    DEV_SET_PWM(100);  // Set backlight brightness
    LCD_1IN28_Init(HORIZONTAL);
    LCD_1IN28_Clear(BLACK);
}

// Draw a line using Bresenham's algorithm
void draw_line(int x0, int y0, int x1, int y1, uint16_t color) {
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1;
    int sy = y0 < y1 ? 1 : -1;
    int err = dx - dy;
    
    while (1) {
        if (x0 >= 0 && x0 < LCD_WIDTH && y0 >= 0 && y0 < LCD_HEIGHT) {
            framebuffer[y0 * LCD_WIDTH + x0] = color;
        }
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

// Rotate vertex using quaternion
void rotate_vertex_quat(Vertex *v, const Quaternion *q) {
    float qw = q->w, qx = q->x, qy = q->y, qz = q->z;
    float vx = v->x, vy = v->y, vz = v->z;
    
    float tx = 2.0f * (qy*vz - qz*vy);
    float ty = 2.0f * (qz*vx - qx*vz);
    float tz = 2.0f * (qx*vy - qy*vx);
    
    v->x = vx + qw*tx + qy*tz - qz*ty;
    v->y = vy + qw*ty + qz*tx - qx*tz;
    v->z = vz + qw*tz + qx*ty - qy*tx;
}

void draw_cube(float pitch __attribute__((unused)), 
               float roll __attribute__((unused)), 
               float yaw __attribute__((unused))) {
    // Clear framebuffer
    for(int i = 0; i < LCD_WIDTH * LCD_HEIGHT; i++) {
        framebuffer[i] = BLACK;
    }
    
    // Define cube vertices
    Vertex cube[8] = {
        {-0.5f, -0.5f, -0.5f}, {0.5f, -0.5f, -0.5f},
        {0.5f, 0.5f, -0.5f}, {-0.5f, 0.5f, -0.5f},
        {-0.5f, -0.5f, 0.5f}, {0.5f, -0.5f, 0.5f},
        {0.5f, 0.5f, 0.5f}, {-0.5f, 0.5f, 0.5f}
    };
    
    // Rotate cube using quaternion
    Vertex rotated[8];
    for (int i = 0; i < 8; i++) {
        rotated[i] = cube[i];
        rotate_vertex_quat(&rotated[i], &kalman.q);
    }
    
    // Define edges
    int edges[12][2] = {
        {0,1}, {1,2}, {2,3}, {3,0},
        {4,5}, {5,6}, {6,7}, {7,4},
        {0,4}, {1,5}, {2,6}, {3,7}
    };
    
    // Draw edges with perspective projection
    int cx = LCD_WIDTH / 2;
    int cy = LCD_HEIGHT / 2;
    
    for (int i = 0; i < 12; i++) {
        const Vertex *v0 = &rotated[edges[i][0]];
        const Vertex *v1 = &rotated[edges[i][1]];
        
        float z0 = v0->z + 1.5f;
        float z1 = v1->z + 1.5f;
        
        if (z0 > 0.1f && z1 > 0.1f) {
            int x0 = cx + (int)(v0->x * 100.0f / z0);
            int y0 = cy + (int)(v0->y * 100.0f / z0);
            int x1 = cx + (int)(v1->x * 100.0f / z1);
            int y1 = cy + (int)(v1->y * 100.0f / z1);
            
            draw_line(x0, y0, x1, y1, GREEN);
        }
    }
}

int main() {
    stdio_init_all();
    sleep_ms(2000);
    
    printf("\n=== 6-DOF Configurable Kalman Filter ===\n");
    printf("Full axis configuration and testing\n");
    printf("Type 'h' for help\n");
    
    // Init I2C
    i2c_init(IMU_I2C_PORT, 400000);
    gpio_set_function(IMU_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(IMU_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(IMU_SDA_PIN);
    gpio_pull_up(IMU_SCL_PIN);
    
    // Init SPI for LCD
    spi_init(LCD_SPI_PORT, 62500000);
    gpio_set_function(LCD_SPI_CLK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(LCD_SPI_MOSI_PIN, GPIO_FUNC_SPI);
    
    lcd_init();
    
    if (!qmi8658_init()) {
        printf("IMU init failed!\n");
        return 1;
    }
    
    kalman_init(&kalman);
    
    uint32_t last_time = time_us_32();
    float filtered_gx = 0, filtered_gy = 0, filtered_gz = 0;
    
    while (1) {
        // Check for commands
        int c = getchar_timeout_us(0);
        if (c != PICO_ERROR_TIMEOUT) {
            process_command((char)c);
        }
        
        // Read raw sensor data
        int16_t raw_ax = i2c_read_register_int16(QMI8658_ADDR, QMI8658_DATA);
        int16_t raw_ay = i2c_read_register_int16(QMI8658_ADDR, QMI8658_DATA + 2);
        int16_t raw_az = i2c_read_register_int16(QMI8658_ADDR, QMI8658_DATA + 4);
        int16_t raw_gx = i2c_read_register_int16(QMI8658_ADDR, QMI8658_DATA + 6);
        int16_t raw_gy = i2c_read_register_int16(QMI8658_ADDR, QMI8658_DATA + 8);
        int16_t raw_gz = i2c_read_register_int16(QMI8658_ADDR, QMI8658_DATA + 10);
        
        // Convert to physical units
        float raw_gyro[3] = {
            (raw_gx / 128.0f) * DEG_TO_RAD,
            (raw_gy / 128.0f) * DEG_TO_RAD,
            (raw_gz / 128.0f) * DEG_TO_RAD
        };
        
        float raw_accel[3] = {
            raw_ax / 16384.0f,
            raw_ay / 16384.0f,
            raw_az / 16384.0f
        };
        
        // Apply axis configuration
        float configured_gyro[3], configured_accel[3];
        apply_axis_config(raw_gyro, configured_gyro, 
                         axis_config.gyro_map, axis_config.gyro_sign, axis_config.gyro_scale);
        apply_axis_config(raw_accel, configured_accel,
                         axis_config.accel_map, axis_config.accel_sign, axis_config.accel_scale);
        
        float gx = configured_gyro[0];
        float gy = configured_gyro[1];
        float gz = configured_gyro[2];
        float ax = configured_accel[0];
        float ay = configured_accel[1];
        float az = configured_accel[2];
        
        // Apply test signals if active
        apply_test_signal(&gx, &gy, &gz, &ax, &ay, &az);
        
        // Filter gyro
        filtered_gx = gyro_filter_alpha * filtered_gx + (1.0f - gyro_filter_alpha) * gx;
        filtered_gy = gyro_filter_alpha * filtered_gy + (1.0f - gyro_filter_alpha) * gy;
        filtered_gz = gyro_filter_alpha * filtered_gz + (1.0f - gyro_filter_alpha) * gz;
        
        // Time delta
        uint32_t current_time = time_us_32();
        float dt = (current_time - last_time) / 1000000.0f;
        last_time = current_time;
        
        // Apply Kalman filter
        float corrected_gx = filtered_gx - kalman.gx_bias;
        float corrected_gy = filtered_gy - kalman.gy_bias;
        float corrected_gz = filtered_gz - kalman.gz_bias;
        
        quat_integrate(&kalman.q, corrected_gx, corrected_gy, corrected_gz, dt);
        
        // Get Euler angles
        float pitch = atan2f(2.0f*(kalman.q.w*kalman.q.x + kalman.q.y*kalman.q.z),
                            1.0f - 2.0f*(kalman.q.x*kalman.q.x + kalman.q.y*kalman.q.y));
        float roll = asinf(2.0f*(kalman.q.w*kalman.q.y - kalman.q.z*kalman.q.x));
        float yaw = atan2f(2.0f*(kalman.q.w*kalman.q.z + kalman.q.x*kalman.q.y),
                          1.0f - 2.0f*(kalman.q.y*kalman.q.y + kalman.q.z*kalman.q.z));
        
        // Debug output
        if (debug_stream) {
            printf("CSV,%lu,%.1f,%.1f,%.1f,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f\n",
                   current_time/1000,
                   (float)raw_gx, (float)raw_gy, (float)raw_gz,
                   ax, ay, az,
                   pitch*RAD_TO_DEG, roll*RAD_TO_DEG, yaw*RAD_TO_DEG);
        }
        
        // Update display
        draw_cube(pitch, roll, yaw);
        LCD_1IN28_Display(framebuffer);
        
        sleep_ms(10);
    }
    
    return 0;
}