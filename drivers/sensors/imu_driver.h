#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#include "system_types.h"
#include "board_config.h"

// IMU configuration
typedef struct {
    uint16_t accel_odr_hz;          // Accelerometer output data rate (Hz)
    uint16_t gyro_odr_hz;           // Gyroscope output data rate (Hz)
    uint8_t accel_range_g;          // Accelerometer range (2, 4, 8, 16 G)
    uint16_t gyro_range_dps;        // Gyroscope range (16, 32, 64, 128, 256, 512, 1024, 2048 DPS)
    bool enable_interrupts;         // Enable interrupt generation
    bool enable_fifo;               // Enable FIFO buffer
} imu_config_t;

// IMU status
typedef struct {
    bool initialized;
    bool data_ready;
    bool fifo_overflow;
    uint16_t fifo_count;
    uint32_t sample_count;
    float temperature;
} imu_status_t;

/**
 * @brief Initialize the IMU driver
 * @return SYSTEM_OK on success, error code on failure
 */
system_result_t imu_init(void);

/**
 * @brief Configure the IMU with specified settings
 * @param config IMU configuration structure
 * @return SYSTEM_OK on success, error code on failure
 */
system_result_t imu_configure(const imu_config_t *config);

/**
 * @brief Read IMU data (accelerometer, gyroscope, temperature)
 * @param data Pointer to IMU data structure
 * @return SYSTEM_OK on success, error code on failure
 */
system_result_t imu_read_data(imu_data_t *data);

/**
 * @brief Read only accelerometer data
 * @param accel Pointer to accelerometer vector
 * @return SYSTEM_OK on success, error code on failure
 */
system_result_t imu_read_accelerometer(vector3_t *accel);

/**
 * @brief Read only gyroscope data
 * @param gyro Pointer to gyroscope vector
 * @return SYSTEM_OK on success, error code on failure
 */
system_result_t imu_read_gyroscope(vector3_t *gyro);

/**
 * @brief Read IMU temperature
 * @param temperature Pointer to temperature value
 * @return SYSTEM_OK on success, error code on failure
 */
system_result_t imu_read_temperature(float *temperature);

/**
 * @brief Get IMU status
 * @param status Pointer to status structure
 * @return SYSTEM_OK on success, error code on failure
 */
system_result_t imu_get_status(imu_status_t *status);

/**
 * @brief Check if new data is available
 * @return true if new data is available, false otherwise
 */
bool imu_data_ready(void);

/**
 * @brief Reset the IMU
 * @return SYSTEM_OK on success, error code on failure
 */
system_result_t imu_reset(void);

/**
 * @brief Put IMU into sleep mode
 * @return SYSTEM_OK on success, error code on failure
 */
system_result_t imu_sleep(void);

/**
 * @brief Wake IMU from sleep mode
 * @return SYSTEM_OK on success, error code on failure
 */
system_result_t imu_wake(void);

/**
 * @brief Set up interrupt callback for data ready
 * @param callback Function to call when data is ready
 * @return SYSTEM_OK on success, error code on failure
 */
system_result_t imu_set_data_ready_callback(void (*callback)(void));

/**
 * @brief Calibrate the IMU (should be called when device is stationary)
 * @return SYSTEM_OK on success, error code on failure
 */
system_result_t imu_calibrate(void);

/**
 * @brief Get device ID
 * @param device_id Pointer to store device ID
 * @return SYSTEM_OK on success, error code on failure
 */
system_result_t imu_get_device_id(uint8_t *device_id);

#endif // IMU_DRIVER_H