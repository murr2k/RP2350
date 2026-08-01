/**
 * QMI8658 6-axis IMU driver.
 *
 * Port of drivers/sensors/qmi8658.h from the RP2350 project. Same chip on both
 * boards, so only the bus layer changed: I2C goes through DEV_I2C_* instead of
 * the Pico SDK.
 *
 * Ranges are fixed at +/-2 g and +/-256 dps, which gives the 16384 LSB/g and
 * 128 LSB/dps scale factors the demos and the host-side test scripts assume.
 */

#ifndef QMI8658_H
#define QMI8658_H

#include <stdbool.h>
#include <stdint.h>

/* Registers */
#define QMI8658_WHO_AM_I    0x00
#define QMI8658_REVISION_ID 0x01
#define QMI8658_CTRL1       0x02
#define QMI8658_CTRL2       0x03
#define QMI8658_CTRL3       0x04
#define QMI8658_CTRL5       0x06
#define QMI8658_CTRL7       0x08
#define QMI8658_CTRL9       0x0A
#define QMI8658_STATUS0     0x2E
#define QMI8658_TEMP_L      0x33
#define QMI8658_AX_L        0x35
#define QMI8658_GX_L        0x3B
#define QMI8658_RESET       0x60

#define QMI8658_DEVICE_ID   0x05

/* Scale factors for the ranges configured by qmi8658_init(). */
#define QMI8658_ACC_LSB_PER_G   16384.0f
#define QMI8658_GYRO_LSB_PER_DPS 128.0f

typedef struct {
    float x;
    float y;
    float z;
} vector3f_t;

/** Probe the sensor and configure it for +/-2 g, +/-256 dps at 250 Hz.
 *  DEV_Module_Init() must have run first. */
bool qmi8658_init(void);

/** True once qmi8658_init() has succeeded. */
bool qmi8658_present(void);

/** I2C address the sensor answered on, 0 if it never did. */
uint8_t qmi8658_address(void);

/** Raw counts, six values read in one burst: ax, ay, az, gx, gy, gz. */
bool qmi8658_read_raw(int16_t *acc_raw, int16_t *gyro_raw);

/** Converted readings: acceleration in g, angular rate in degrees per second.
 *  Calibration offsets from qmi8658_calibrate() are subtracted from the gyro. */
bool qmi8658_read(vector3f_t *acc, vector3f_t *gyro);

/** Die temperature in degrees Celsius. */
float qmi8658_read_temperature(void);

/** Average the gyro at rest and store the result as the zero-rate offset. */
void qmi8658_calibrate(uint16_t samples);

/** Current gyro zero-rate offset in degrees per second. */
vector3f_t qmi8658_gyro_offset(void);
void qmi8658_set_gyro_offset(vector3f_t offset);

/** True when a new sample set is ready. */
bool qmi8658_data_ready(void);

#endif /* QMI8658_H */
