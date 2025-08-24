/**
 * Configurable Madgwick Cube with CLI Menu
 * Runtime adjustable axis mapping and parameters via serial interface
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

// QMI8658 registers
#define QMI8658_ADDR    0x6B
#define QMI8658_WHO_AM_I        0x00
#define QMI8658_CTRL1           0x02
#define QMI8658_CTRL2           0x03
#define QMI8658_CTRL3           0x04
#define QMI8658_CTRL7           0x08
#define QMI8658_AX_L            0x35
#define QMI8658_GX_L            0x3B

// Configuration structure
typedef struct {
    // Axis inversions
    bool invert_ax, invert_ay, invert_az;
    bool invert_gx, invert_gy, invert_gz;
    
    // Axis swaps (0=X, 1=Y, 2=Z)
    int accel_map[3];  // Maps sensor axis to output axis
    int gyro_map[3];
    
    // Filter parameters
    float beta;        // Madgwick gain
    float sample_rate; // Hz
    
    // Display options
    bool show_axes;
    bool show_data;
    bool show_cube;
    bool paused;
} config_t;

// Global configuration
static config_t config = {
    .invert_ax = false, .invert_ay = false, .invert_az = false,
    .invert_gx = false, .invert_gy = true, .invert_gz = false,  // Y inverted based on testing
    .accel_map = {0, 1, 2},  // No swapping
    .gyro_map = {0, 1, 2},
    .beta = 0.1f,
    .sample_rate = 100.0f,
    .show_axes = true,
    .show_data = true,
    .show_cube = true,
    .paused = false
};

// Frame buffer
static uint16_t frame_buffer[WIDTH * HEIGHT];

// Quaternion
typedef struct {
    float w, x, y, z;
} quaternion_t;

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
    {0,1}, {1,2}, {2,3}, {3,0},
    {4,5}, {5,6}, {6,7}, {7,4},
    {0,4}, {1,5}, {2,6}, {3,7}
};

// Gyro bias
static float gx_bias = 0, gy_bias = 0, gz_bias = 0;

// Last sensor values for display
static float last_ax = 0, last_ay = 0, last_az = 0;
static float last_gx = 0, last_gy = 0, last_gz = 0;

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
    i2c_write_register(QMI8658_ADDR, QMI8658_CTRL2, 0x04);
    i2c_write_register(QMI8658_ADDR, QMI8658_CTRL3, 0x54);
    i2c_write_register(QMI8658_ADDR, QMI8658_CTRL7, 0x03);
    
    return true;
}

// Read IMU with configurable mapping
static void read_imu(float *ax, float *ay, float *az, float *gx, float *gy, float *gz) {
    int16_t raw_ax = i2c_read_16bit(QMI8658_ADDR, QMI8658_AX_L);
    int16_t raw_ay = i2c_read_16bit(QMI8658_ADDR, QMI8658_AX_L + 2);
    int16_t raw_az = i2c_read_16bit(QMI8658_ADDR, QMI8658_AX_L + 4);
    
    int16_t raw_gx = i2c_read_16bit(QMI8658_ADDR, QMI8658_GX_L);
    int16_t raw_gy = i2c_read_16bit(QMI8658_ADDR, QMI8658_GX_L + 2);
    int16_t raw_gz = i2c_read_16bit(QMI8658_ADDR, QMI8658_GX_L + 4);
    
    // Convert to physical units
    float accel[3] = {
        raw_ax / 16384.0f,
        raw_ay / 16384.0f,
        raw_az / 16384.0f
    };
    
    float gyro[3] = {
        (raw_gx / 128.0f - gx_bias) * (PI / 180.0f),
        (raw_gy / 128.0f - gy_bias) * (PI / 180.0f),
        (raw_gz / 128.0f - gz_bias) * (PI / 180.0f)
    };
    
    // Apply inversions
    if (config.invert_ax) accel[0] = -accel[0];
    if (config.invert_ay) accel[1] = -accel[1];
    if (config.invert_az) accel[2] = -accel[2];
    
    if (config.invert_gx) gyro[0] = -gyro[0];
    if (config.invert_gy) gyro[1] = -gyro[1];
    if (config.invert_gz) gyro[2] = -gyro[2];
    
    // Apply axis mapping
    *ax = accel[config.accel_map[0]];
    *ay = accel[config.accel_map[1]];
    *az = accel[config.accel_map[2]];
    
    *gx = gyro[config.gyro_map[0]];
    *gy = gyro[config.gyro_map[1]];
    *gz = gyro[config.gyro_map[2]];
    
    // Store for display
    last_ax = *ax; last_ay = *ay; last_az = *az;
    last_gx = *gx; last_gy = *gy; last_gz = *gz;
}

// Fast inverse square root
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
        
        // Auxiliary variables
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
        qDot1 -= config.beta * s0;
        qDot2 -= config.beta * s1;
        qDot3 -= config.beta * s2;
        qDot4 -= config.beta * s3;
    }
    
    // Integrate rate of change of quaternion
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
    quaternion_t p = {0, v.x, v.y, v.z};
    quaternion_t q_conj = {q.w, -q.x, -q.y, -q.z};
    
    quaternion_t qp;
    qp.w = q.w * p.w - q.x * p.x - q.y * p.y - q.z * p.z;
    qp.x = q.w * p.x + q.x * p.w + q.y * p.z - q.z * p.y;
    qp.y = q.w * p.y - q.x * p.z + q.y * p.w + q.z * p.x;
    qp.z = q.w * p.z + q.x * p.y - q.y * p.x + q.z * p.w;
    
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
    
    for (int i = 0; i < 8; i++) {
        project(vertices[i], &screen[i][0], &screen[i][1]);
    }
    
    for (int i = 0; i < 12; i++) {
        int v0 = edges[i][0];
        int v1 = edges[i][1];
        
        uint16_t color = 0xFFFF;
        if (i < 4) color = 0xF800;
        else if (i < 8) color = 0x07E0;
        else color = 0x001F;
        
        draw_line(screen[v0][0], screen[v0][1], 
                 screen[v1][0], screen[v1][1], color);
    }
}

// Draw axes
void draw_axes(quaternion_t orientation) {
    vertex_t x_axis = {40, 0, 0};
    vertex_t y_axis = {0, 40, 0};
    vertex_t z_axis = {0, 0, 40};
    vertex_t origin = {0, 0, 0};
    
    vertex_t x_rot = rotate_by_quaternion(x_axis, orientation);
    vertex_t y_rot = rotate_by_quaternion(y_axis, orientation);
    vertex_t z_rot = rotate_by_quaternion(z_axis, orientation);
    
    int ox, oy, xx, xy, yx, yy, zx, zy;
    project(origin, &ox, &oy);
    project(x_rot, &xx, &xy);
    project(y_rot, &yx, &yy);
    project(z_rot, &zx, &zy);
    
    draw_line(ox, oy, xx, xy, 0xF800);  // X = Red
    draw_line(ox, oy, yx, yy, 0x07E0);  // Y = Green
    draw_line(ox, oy, zx, zy, 0x001F);  // Z = Blue
}

// Draw sensor data
void draw_sensor_data(void) {
    // Draw data bars at top
    int ax_bar = (int)(fabsf(last_ax) * 30);
    int ay_bar = (int)(fabsf(last_ay) * 30);
    int az_bar = (int)(fabsf(last_az) * 30);
    
    // Accel bars
    if (last_ax > 0) draw_line(10, 10, 10 + ax_bar, 10, 0xF800);
    else draw_line(10, 10, 10 - ax_bar, 10, 0xF880);
    
    if (last_ay > 0) draw_line(10, 15, 10 + ay_bar, 15, 0x07E0);
    else draw_line(10, 15, 10 - ay_bar, 15, 0x07F0);
    
    if (last_az > 0) draw_line(10, 20, 10 + az_bar, 20, 0x001F);
    else draw_line(10, 20, 10 - az_bar, 20, 0x001F);
    
    // Gyro bars at bottom
    int gx_bar = (int)(fabsf(last_gx) * 20);
    int gy_bar = (int)(fabsf(last_gy) * 20);
    int gz_bar = (int)(fabsf(last_gz) * 20);
    
    if (last_gx > 0) draw_line(10, 220, 10 + gx_bar, 220, 0xF800);
    else draw_line(10, 220, 10 - gx_bar, 220, 0xF880);
    
    if (last_gy > 0) draw_line(10, 225, 10 + gy_bar, 225, 0x07E0);
    else draw_line(10, 225, 10 - gy_bar, 225, 0x07F0);
    
    if (last_gz > 0) draw_line(10, 230, 10 + gz_bar, 230, 0x001F);
    else draw_line(10, 230, 10 - gz_bar, 230, 0x001F);
}

// Print menu
void print_menu(void) {
    printf("\n\n=== Configurable Madgwick Cube Menu ===\n");
    printf("Axis Controls:\n");
    printf("  1-6: Toggle axis inversions (1=AX 2=AY 3=AZ 4=GX 5=GY 6=GZ)\n");
    printf("  q/w: Swap accel X axis (q=swap with Y, w=swap with Z)\n");
    printf("  a/s: Swap accel Y axis (a=swap with X, s=swap with Z)\n");
    printf("  z/x: Swap accel Z axis (z=swap with X, x=swap with Y)\n");
    printf("\nDisplay Controls:\n");
    printf("  d: Toggle sensor data display\n");
    printf("  c: Toggle cube display\n");
    printf("  e: Toggle axes display\n");
    printf("  p: Pause/unpause\n");
    printf("\nFilter Controls:\n");
    printf("  +/-: Increase/decrease beta (current: %.3f)\n", config.beta);
    printf("  r: Reset orientation\n");
    printf("  b: Recalibrate gyro bias\n");
    printf("\nOther:\n");
    printf("  v: View current sensor values\n");
    printf("  m: Show this menu\n");
    printf("  i: Show current axis configuration\n");
    printf("\nCurrent inversions: AX:%c AY:%c AZ:%c GX:%c GY:%c GZ:%c\n",
           config.invert_ax ? 'Y' : 'N', config.invert_ay ? 'Y' : 'N', 
           config.invert_az ? 'Y' : 'N', config.invert_gx ? 'Y' : 'N',
           config.invert_gy ? 'Y' : 'N', config.invert_gz ? 'Y' : 'N');
}

// Calibrate gyro
void calibrate_gyro(void) {
    printf("Calibrating gyro... hold still\n");
    gx_bias = gy_bias = gz_bias = 0;
    
    float gx_sum = 0, gy_sum = 0, gz_sum = 0;
    for (int i = 0; i < 200; i++) {
        int16_t raw_gx = i2c_read_16bit(QMI8658_ADDR, QMI8658_GX_L);
        int16_t raw_gy = i2c_read_16bit(QMI8658_ADDR, QMI8658_GX_L + 2);
        int16_t raw_gz = i2c_read_16bit(QMI8658_ADDR, QMI8658_GX_L + 4);
        
        gx_sum += raw_gx / 128.0f;
        gy_sum += raw_gy / 128.0f;
        gz_sum += raw_gz / 128.0f;
        sleep_ms(10);
    }
    
    gx_bias = gx_sum / 200.0f;
    gy_bias = gy_sum / 200.0f;
    gz_bias = gz_sum / 200.0f;
    
    printf("Calibration complete: GX:%.2f GY:%.2f GZ:%.2f\n", gx_bias, gy_bias, gz_bias);
}

// Process serial command
void process_command(char cmd) {
    switch (cmd) {
        // Axis inversions
        case '1': config.invert_ax = !config.invert_ax; 
                  printf("AX inversion: %s\n", config.invert_ax ? "ON" : "OFF"); break;
        case '2': config.invert_ay = !config.invert_ay; 
                  printf("AY inversion: %s\n", config.invert_ay ? "ON" : "OFF"); break;
        case '3': config.invert_az = !config.invert_az; 
                  printf("AZ inversion: %s\n", config.invert_az ? "ON" : "OFF"); break;
        case '4': config.invert_gx = !config.invert_gx; 
                  printf("GX inversion: %s\n", config.invert_gx ? "ON" : "OFF"); break;
        case '5': config.invert_gy = !config.invert_gy; 
                  printf("GY inversion: %s\n", config.invert_gy ? "ON" : "OFF"); break;
        case '6': config.invert_gz = !config.invert_gz; 
                  printf("GZ inversion: %s\n", config.invert_gz ? "ON" : "OFF"); break;
        
        // Display toggles
        case 'd': config.show_data = !config.show_data; 
                  printf("Data display: %s\n", config.show_data ? "ON" : "OFF"); break;
        case 'c': config.show_cube = !config.show_cube; 
                  printf("Cube display: %s\n", config.show_cube ? "ON" : "OFF"); break;
        case 'e': config.show_axes = !config.show_axes; 
                  printf("Axes display: %s\n", config.show_axes ? "ON" : "OFF"); break;
        case 'p': config.paused = !config.paused; 
                  printf("Display %s\n", config.paused ? "PAUSED" : "RUNNING"); break;
        
        // Filter parameters
        case '+': config.beta += 0.01f; if (config.beta > 1.0f) config.beta = 1.0f;
                  printf("Beta: %.3f\n", config.beta); break;
        case '-': config.beta -= 0.01f; if (config.beta < 0.0f) config.beta = 0.0f;
                  printf("Beta: %.3f\n", config.beta); break;
        
        // Reset and calibrate
        case 'r': q.w = 1.0f; q.x = q.y = q.z = 0.0f; 
                  printf("Orientation reset\n"); break;
        case 'b': calibrate_gyro(); break;
        
        // Info
        case 'v': printf("Sensors: AX:%.3f AY:%.3f AZ:%.3f GX:%.3f GY:%.3f GZ:%.3f\n",
                        last_ax, last_ay, last_az, last_gx, last_gy, last_gz); break;
        case 'm': print_menu(); break;
        case 'i': printf("Config: AX:%c AY:%c AZ:%c GX:%c GY:%c GZ:%c Beta:%.3f\n",
                        config.invert_ax ? '-' : '+', config.invert_ay ? '-' : '+',
                        config.invert_az ? '-' : '+', config.invert_gx ? '-' : '+',
                        config.invert_gy ? '-' : '+', config.invert_gz ? '-' : '+',
                        config.beta); break;
    }
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    
    printf("\n=== Configurable Madgwick Cube ===\n");
    printf("Press 'm' for menu\n\n");
    
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
    
    // Calibrate
    calibrate_gyro();
    
    uint32_t last_time = time_us_32();
    
    print_menu();
    
    while (1) {
        // Check for serial input
        int c = getchar_timeout_us(0);
        if (c != PICO_ERROR_TIMEOUT) {
            process_command((char)c);
        }
        
        if (!config.paused) {
            // Read IMU
            float ax, ay, az, gx, gy, gz;
            read_imu(&ax, &ay, &az, &gx, &gy, &gz);
            
            // Calculate dt
            uint32_t now = time_us_32();
            float dt = (now - last_time) / 1000000.0f;
            last_time = now;
            
            // Update filter
            madgwick_update(gx, gy, gz, ax, ay, az, dt);
            
            // Clear buffer
            clear_buffer(0x0000);
            
            // Draw based on settings
            if (config.show_cube) {
                vertex_t rotated[8];
                for (int i = 0; i < 8; i++) {
                    rotated[i] = rotate_by_quaternion(cube[i], q);
                }
                draw_cube(rotated);
            }
            
            if (config.show_axes) {
                draw_axes(q);
            }
            
            if (config.show_data) {
                draw_sensor_data();
            }
            
            // Display
            display_buffer();
        }
        
        // Target rate
        sleep_ms(10);
    }
    
    return 0;
}