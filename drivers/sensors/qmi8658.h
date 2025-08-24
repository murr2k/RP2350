/**
 * QMI8658 6-axis IMU Driver Header
 * For Waveshare RP2350-LCD-1.28
 */

#ifndef QMI8658_H
#define QMI8658_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/i2c.h"

// I2C addresses
#define QMI8658_ADDR_LOW    0x6A  // SA0 = 0
#define QMI8658_ADDR_HIGH   0x6B  // SA0 = 1

// Vector structure for 3D data
typedef struct {
    float x;
    float y;
    float z;
} vector3f_t;

// QMI8658 device structure
typedef struct {
    i2c_inst_t *i2c_inst;
    uint8_t addr;
    uint8_t sda_pin;
    uint8_t scl_pin;
    float acc_scale;
    float gyro_scale;
    vector3f_t acc_offset;
    vector3f_t gyro_offset;
} qmi8658_t;

/**
 * Initialize the QMI8658 sensor
 * @param i2c I2C instance (i2c0 or i2c1)
 * @param addr I2C address of the sensor
 * @param sda_pin GPIO pin for SDA
 * @param scl_pin GPIO pin for SCL
 * @return true if successful, false otherwise
 */
bool qmi8658_init(i2c_inst_t *i2c, uint8_t addr, uint8_t sda_pin, uint8_t scl_pin);

/**
 * Read raw sensor data
 * @param acc_raw Array to store raw accelerometer data [x, y, z]
 * @param gyro_raw Array to store raw gyroscope data [x, y, z]
 * @return true if successful, false otherwise
 */
bool qmi8658_read_raw(int16_t *acc_raw, int16_t *gyro_raw);

/**
 * Read calibrated sensor data
 * @param acc Pointer to store accelerometer data in g
 * @param gyro Pointer to store gyroscope data in degrees/sec
 * @return true if successful, false otherwise
 */
bool qmi8658_read(vector3f_t *acc, vector3f_t *gyro);

/**
 * Read temperature from sensor
 * @return Temperature in Celsius
 */
float qmi8658_read_temperature(void);

/**
 * Calibrate the sensor (should be done with device at rest)
 * @param samples Number of samples to average for calibration
 */
void qmi8658_calibrate(uint16_t samples);

/**
 * Check if new data is ready
 * @return true if data is ready, false otherwise
 */
bool qmi8658_data_ready(void);

/**
 * Set measurement ranges
 * @param acc_range Accelerometer range in g (2, 4, 8, or 16)
 * @param gyro_range Gyroscope range in dps (16, 32, 64, 128, 256, 512, 1024, or 2048)
 */
void qmi8658_set_range(uint8_t acc_range, uint16_t gyro_range);

/**
 * Set output data rates
 * @param acc_odr Accelerometer ODR in Hz (31, 62, 125, 250, 500, 1000, 2000, 4000, 8000)
 * @param gyro_odr Gyroscope ODR in Hz (31, 62, 125, 250, 500, 1000, 2000, 4000, 8000)
 */
void qmi8658_set_odr(uint16_t acc_odr, uint16_t gyro_odr);

#endif // QMI8658_H