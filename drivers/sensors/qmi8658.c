/**
 * QMI8658 6-axis IMU Driver Implementation
 * For Waveshare RP2350-LCD-1.28
 */

#include "qmi8658.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include <string.h>

// QMI8658 Register Addresses
#define QMI8658_WHO_AM_I        0x00
#define QMI8658_REVISION_ID     0x01
#define QMI8658_CTRL1           0x02
#define QMI8658_CTRL2           0x03
#define QMI8658_CTRL3           0x04
#define QMI8658_CTRL4           0x05
#define QMI8658_CTRL5           0x06
#define QMI8658_CTRL6           0x07
#define QMI8658_CTRL7           0x08
#define QMI8658_CTRL8           0x09
#define QMI8658_CTRL9           0x0A
#define QMI8658_STATUS0         0x2E
#define QMI8658_STATUS1         0x2F
#define QMI8658_TEMP_L          0x33
#define QMI8658_TEMP_H          0x34
#define QMI8658_AX_L            0x35
#define QMI8658_AX_H            0x36
#define QMI8658_AY_L            0x37
#define QMI8658_AY_H            0x38
#define QMI8658_AZ_L            0x39
#define QMI8658_AZ_H            0x3A
#define QMI8658_GX_L            0x3B
#define QMI8658_GX_H            0x3C
#define QMI8658_GY_L            0x3D
#define QMI8658_GY_H            0x3E
#define QMI8658_GZ_L            0x3F
#define QMI8658_GZ_H            0x40
#define QMI8658_RESET           0x60

// Expected WHO_AM_I value
#define QMI8658_DEVICE_ID       0x05

// Control register values
#define CTRL1_ACC_ENABLE        0x01
#define CTRL1_GYRO_ENABLE       0x02
#define CTRL1_ACC_FS_2G         0x00
#define CTRL1_ACC_FS_4G         0x08
#define CTRL1_ACC_FS_8G         0x10
#define CTRL1_ACC_FS_16G        0x18
#define CTRL1_GYRO_FS_16DPS     0x00
#define CTRL1_GYRO_FS_32DPS     0x20
#define CTRL1_GYRO_FS_64DPS     0x40
#define CTRL1_GYRO_FS_128DPS    0x60
#define CTRL1_GYRO_FS_256DPS    0x80
#define CTRL1_GYRO_FS_512DPS    0xA0
#define CTRL1_GYRO_FS_1024DPS   0xC0
#define CTRL1_GYRO_FS_2048DPS   0xE0

#define CTRL2_ACC_ODR_8000Hz    0x00
#define CTRL2_ACC_ODR_4000Hz    0x01
#define CTRL2_ACC_ODR_2000Hz    0x02
#define CTRL2_ACC_ODR_1000Hz    0x03
#define CTRL2_ACC_ODR_500Hz     0x04
#define CTRL2_ACC_ODR_250Hz     0x05
#define CTRL2_ACC_ODR_125Hz     0x06
#define CTRL2_ACC_ODR_62_5Hz    0x07
#define CTRL2_ACC_ODR_31_25Hz   0x08

#define CTRL3_GYRO_ODR_8000Hz   0x00
#define CTRL3_GYRO_ODR_4000Hz   0x10
#define CTRL3_GYRO_ODR_2000Hz   0x20
#define CTRL3_GYRO_ODR_1000Hz   0x30
#define CTRL3_GYRO_ODR_500Hz    0x40
#define CTRL3_GYRO_ODR_250Hz    0x50
#define CTRL3_GYRO_ODR_125Hz    0x60
#define CTRL3_GYRO_ODR_62_5Hz   0x70
#define CTRL3_GYRO_ODR_31_25Hz  0x80

// Static instance
static qmi8658_t g_qmi8658;

// Helper functions
static int qmi8658_write_reg(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    return i2c_write_blocking(g_qmi8658.i2c_inst, g_qmi8658.addr, data, 2, false);
}

static int qmi8658_read_reg(uint8_t reg, uint8_t *value) {
    if (i2c_write_blocking(g_qmi8658.i2c_inst, g_qmi8658.addr, &reg, 1, true) < 0)
        return -1;
    if (i2c_read_blocking(g_qmi8658.i2c_inst, g_qmi8658.addr, value, 1, false) < 0)
        return -1;
    return 0;
}

static int qmi8658_read_regs(uint8_t reg, uint8_t *buffer, size_t len) {
    if (i2c_write_blocking(g_qmi8658.i2c_inst, g_qmi8658.addr, &reg, 1, true) < 0)
        return -1;
    if (i2c_read_blocking(g_qmi8658.i2c_inst, g_qmi8658.addr, buffer, len, false) < 0)
        return -1;
    return 0;
}

bool qmi8658_init(i2c_inst_t *i2c, uint8_t addr, uint8_t sda_pin, uint8_t scl_pin) {
    // Store configuration
    g_qmi8658.i2c_inst = i2c;
    g_qmi8658.addr = addr;
    g_qmi8658.sda_pin = sda_pin;
    g_qmi8658.scl_pin = scl_pin;
    
    // Initialize I2C
    i2c_init(i2c, 400 * 1000); // 400kHz
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);
    
    // Wait for sensor to stabilize
    sleep_ms(100);
    
    // Check device ID
    uint8_t who_am_i;
    if (qmi8658_read_reg(QMI8658_WHO_AM_I, &who_am_i) < 0) {
        printf("QMI8658: Failed to read WHO_AM_I\n");
        return false;
    }
    
    if (who_am_i != QMI8658_DEVICE_ID) {
        printf("QMI8658: Invalid device ID: 0x%02X (expected 0x%02X)\n", who_am_i, QMI8658_DEVICE_ID);
        return false;
    }
    
    // Software reset
    qmi8658_write_reg(QMI8658_RESET, 0xB0);
    sleep_ms(50);
    
    // Configure sensor
    // CTRL1: Enable ACC and GYRO, set full scale ranges
    qmi8658_write_reg(QMI8658_CTRL1, CTRL1_ACC_ENABLE | CTRL1_GYRO_ENABLE | 
                      CTRL1_ACC_FS_8G | CTRL1_GYRO_FS_512DPS);
    
    // CTRL2: Set accelerometer ODR to 250Hz
    qmi8658_write_reg(QMI8658_CTRL2, CTRL2_ACC_ODR_250Hz);
    
    // CTRL3: Set gyroscope ODR to 250Hz
    qmi8658_write_reg(QMI8658_CTRL3, CTRL3_GYRO_ODR_250Hz);
    
    // CTRL5: Enable data ready interrupt
    qmi8658_write_reg(QMI8658_CTRL5, 0x01);
    
    // CTRL7: Enable sensors
    qmi8658_write_reg(QMI8658_CTRL7, 0x03);
    
    // Store configuration
    g_qmi8658.acc_scale = 8.0f / 32768.0f;   // ±8g
    g_qmi8658.gyro_scale = 512.0f / 32768.0f; // ±512dps
    
    // Initialize calibration offsets to zero
    memset(&g_qmi8658.acc_offset, 0, sizeof(vector3f_t));
    memset(&g_qmi8658.gyro_offset, 0, sizeof(vector3f_t));
    
    printf("QMI8658: Initialized successfully\n");
    return true;
}

bool qmi8658_read_raw(int16_t *acc_raw, int16_t *gyro_raw) {
    uint8_t buffer[12];
    
    // Read all sensor data at once (6 bytes acc + 6 bytes gyro)
    if (qmi8658_read_regs(QMI8658_AX_L, buffer, 12) < 0) {
        return false;
    }
    
    // Parse accelerometer data (little-endian)
    acc_raw[0] = (int16_t)(buffer[1] << 8 | buffer[0]);
    acc_raw[1] = (int16_t)(buffer[3] << 8 | buffer[2]);
    acc_raw[2] = (int16_t)(buffer[5] << 8 | buffer[4]);
    
    // Parse gyroscope data (little-endian)
    gyro_raw[0] = (int16_t)(buffer[7] << 8 | buffer[6]);
    gyro_raw[1] = (int16_t)(buffer[9] << 8 | buffer[8]);
    gyro_raw[2] = (int16_t)(buffer[11] << 8 | buffer[10]);
    
    return true;
}

bool qmi8658_read(vector3f_t *acc, vector3f_t *gyro) {
    int16_t acc_raw[3], gyro_raw[3];
    
    if (!qmi8658_read_raw(acc_raw, gyro_raw)) {
        return false;
    }
    
    // Convert to physical units and apply calibration
    acc->x = acc_raw[0] * g_qmi8658.acc_scale - g_qmi8658.acc_offset.x;
    acc->y = acc_raw[1] * g_qmi8658.acc_scale - g_qmi8658.acc_offset.y;
    acc->z = acc_raw[2] * g_qmi8658.acc_scale - g_qmi8658.acc_offset.z;
    
    gyro->x = gyro_raw[0] * g_qmi8658.gyro_scale - g_qmi8658.gyro_offset.x;
    gyro->y = gyro_raw[1] * g_qmi8658.gyro_scale - g_qmi8658.gyro_offset.y;
    gyro->z = gyro_raw[2] * g_qmi8658.gyro_scale - g_qmi8658.gyro_offset.z;
    
    return true;
}

float qmi8658_read_temperature(void) {
    uint8_t temp_data[2];
    
    if (qmi8658_read_regs(QMI8658_TEMP_L, temp_data, 2) < 0) {
        return 0.0f;
    }
    
    int16_t temp_raw = (int16_t)(temp_data[1] << 8 | temp_data[0]);
    
    // Convert to Celsius (formula from datasheet)
    return temp_raw / 256.0f + 25.0f;
}

void qmi8658_calibrate(uint16_t samples) {
    vector3f_t acc_sum = {0, 0, 0};
    vector3f_t gyro_sum = {0, 0, 0};
    vector3f_t acc, gyro;
    
    printf("QMI8658: Starting calibration with %d samples...\n", samples);
    printf("QMI8658: Keep the device still on a flat surface\n");
    sleep_ms(2000);
    
    // Collect samples
    for (uint16_t i = 0; i < samples; i++) {
        if (qmi8658_read(&acc, &gyro)) {
            acc_sum.x += acc.x;
            acc_sum.y += acc.y;
            acc_sum.z += acc.z;
            gyro_sum.x += gyro.x;
            gyro_sum.y += gyro.y;
            gyro_sum.z += gyro.z;
        }
        sleep_ms(10);
        
        if (i % 100 == 0) {
            printf(".");
        }
    }
    printf("\n");
    
    // Calculate averages
    g_qmi8658.acc_offset.x = acc_sum.x / samples;
    g_qmi8658.acc_offset.y = acc_sum.y / samples;
    g_qmi8658.acc_offset.z = (acc_sum.z / samples) - 1.0f; // Subtract 1g for Z axis
    
    g_qmi8658.gyro_offset.x = gyro_sum.x / samples;
    g_qmi8658.gyro_offset.y = gyro_sum.y / samples;
    g_qmi8658.gyro_offset.z = gyro_sum.z / samples;
    
    printf("QMI8658: Calibration complete\n");
    printf("  Acc offset: %.3f, %.3f, %.3f\n", 
           g_qmi8658.acc_offset.x, g_qmi8658.acc_offset.y, g_qmi8658.acc_offset.z);
    printf("  Gyro offset: %.3f, %.3f, %.3f\n", 
           g_qmi8658.gyro_offset.x, g_qmi8658.gyro_offset.y, g_qmi8658.gyro_offset.z);
}

bool qmi8658_data_ready(void) {
    uint8_t status;
    if (qmi8658_read_reg(QMI8658_STATUS0, &status) < 0) {
        return false;
    }
    return (status & 0x03) == 0x03; // Both acc and gyro data ready
}

void qmi8658_set_range(uint8_t acc_range, uint16_t gyro_range) {
    uint8_t ctrl1 = CTRL1_ACC_ENABLE | CTRL1_GYRO_ENABLE;
    
    // Set accelerometer range
    switch (acc_range) {
        case 2:
            ctrl1 |= CTRL1_ACC_FS_2G;
            g_qmi8658.acc_scale = 2.0f / 32768.0f;
            break;
        case 4:
            ctrl1 |= CTRL1_ACC_FS_4G;
            g_qmi8658.acc_scale = 4.0f / 32768.0f;
            break;
        case 8:
            ctrl1 |= CTRL1_ACC_FS_8G;
            g_qmi8658.acc_scale = 8.0f / 32768.0f;
            break;
        case 16:
            ctrl1 |= CTRL1_ACC_FS_16G;
            g_qmi8658.acc_scale = 16.0f / 32768.0f;
            break;
        default:
            ctrl1 |= CTRL1_ACC_FS_8G;
            g_qmi8658.acc_scale = 8.0f / 32768.0f;
    }
    
    // Set gyroscope range
    switch (gyro_range) {
        case 16:
            ctrl1 |= CTRL1_GYRO_FS_16DPS;
            g_qmi8658.gyro_scale = 16.0f / 32768.0f;
            break;
        case 32:
            ctrl1 |= CTRL1_GYRO_FS_32DPS;
            g_qmi8658.gyro_scale = 32.0f / 32768.0f;
            break;
        case 64:
            ctrl1 |= CTRL1_GYRO_FS_64DPS;
            g_qmi8658.gyro_scale = 64.0f / 32768.0f;
            break;
        case 128:
            ctrl1 |= CTRL1_GYRO_FS_128DPS;
            g_qmi8658.gyro_scale = 128.0f / 32768.0f;
            break;
        case 256:
            ctrl1 |= CTRL1_GYRO_FS_256DPS;
            g_qmi8658.gyro_scale = 256.0f / 32768.0f;
            break;
        case 512:
            ctrl1 |= CTRL1_GYRO_FS_512DPS;
            g_qmi8658.gyro_scale = 512.0f / 32768.0f;
            break;
        case 1024:
            ctrl1 |= CTRL1_GYRO_FS_1024DPS;
            g_qmi8658.gyro_scale = 1024.0f / 32768.0f;
            break;
        case 2048:
            ctrl1 |= CTRL1_GYRO_FS_2048DPS;
            g_qmi8658.gyro_scale = 2048.0f / 32768.0f;
            break;
        default:
            ctrl1 |= CTRL1_GYRO_FS_512DPS;
            g_qmi8658.gyro_scale = 512.0f / 32768.0f;
    }
    
    qmi8658_write_reg(QMI8658_CTRL1, ctrl1);
}

void qmi8658_set_odr(uint16_t acc_odr, uint16_t gyro_odr) {
    uint8_t ctrl2 = 0, ctrl3 = 0;
    
    // Set accelerometer ODR
    switch (acc_odr) {
        case 8000: ctrl2 = CTRL2_ACC_ODR_8000Hz; break;
        case 4000: ctrl2 = CTRL2_ACC_ODR_4000Hz; break;
        case 2000: ctrl2 = CTRL2_ACC_ODR_2000Hz; break;
        case 1000: ctrl2 = CTRL2_ACC_ODR_1000Hz; break;
        case 500:  ctrl2 = CTRL2_ACC_ODR_500Hz; break;
        case 250:  ctrl2 = CTRL2_ACC_ODR_250Hz; break;
        case 125:  ctrl2 = CTRL2_ACC_ODR_125Hz; break;
        case 62:   ctrl2 = CTRL2_ACC_ODR_62_5Hz; break;
        case 31:   ctrl2 = CTRL2_ACC_ODR_31_25Hz; break;
        default:   ctrl2 = CTRL2_ACC_ODR_250Hz; break;
    }
    
    // Set gyroscope ODR
    switch (gyro_odr) {
        case 8000: ctrl3 = CTRL3_GYRO_ODR_8000Hz; break;
        case 4000: ctrl3 = CTRL3_GYRO_ODR_4000Hz; break;
        case 2000: ctrl3 = CTRL3_GYRO_ODR_2000Hz; break;
        case 1000: ctrl3 = CTRL3_GYRO_ODR_1000Hz; break;
        case 500:  ctrl3 = CTRL3_GYRO_ODR_500Hz; break;
        case 250:  ctrl3 = CTRL3_GYRO_ODR_250Hz; break;
        case 125:  ctrl3 = CTRL3_GYRO_ODR_125Hz; break;
        case 62:   ctrl3 = CTRL3_GYRO_ODR_62_5Hz; break;
        case 31:   ctrl3 = CTRL3_GYRO_ODR_31_25Hz; break;
        default:   ctrl3 = CTRL3_GYRO_ODR_250Hz; break;
    }
    
    qmi8658_write_reg(QMI8658_CTRL2, ctrl2);
    qmi8658_write_reg(QMI8658_CTRL3, ctrl3);
}