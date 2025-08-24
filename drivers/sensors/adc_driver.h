#ifndef ADC_DRIVER_H
#define ADC_DRIVER_H

#include "system_types.h"
#include "board_config.h"

// ADC channel definitions
typedef enum {
    ADC_CHANNEL_VBAT = 0,    // Battery voltage
    ADC_CHANNEL_TEMP = 1,    // External temperature sensor
    ADC_CHANNEL_LIGHT = 2,   // Light sensor
    ADC_CHANNEL_INTERNAL_TEMP = 4,  // Internal temperature sensor
    ADC_CHANNEL_COUNT = 5
} adc_channel_t;

// ADC configuration
typedef struct {
    uint16_t sample_rate_hz;    // Sampling rate in Hz
    uint8_t oversampling;       // Oversampling factor (1, 2, 4, 8, 16, 32, 64, 128, 256)
    bool continuous_mode;       // Enable continuous sampling
} adc_config_t;

// ADC reading with metadata
typedef struct {
    uint16_t raw_value;         // Raw ADC value (0-4095)
    float voltage;              // Converted voltage
    float scaled_value;         // Scaled/calibrated value
    uint64_t timestamp_us;      // Timestamp of reading
    bool valid;                 // Reading validity
} adc_reading_t;

/**
 * @brief Initialize the ADC driver
 * @return SYSTEM_OK on success, error code on failure
 */
system_result_t adc_driver_init(void);

/**
 * @brief Configure ADC settings
 * @param config ADC configuration structure
 * @return SYSTEM_OK on success, error code on failure
 */
system_result_t adc_configure(const adc_config_t *config);

/**
 * @brief Read a single ADC channel
 * @param channel ADC channel to read
 * @param reading Pointer to store reading result
 * @return SYSTEM_OK on success, error code on failure
 */
system_result_t adc_read_channel(adc_channel_t channel, adc_reading_t *reading);

/**
 * @brief Read battery voltage
 * @param voltage_mv Pointer to store voltage in millivolts
 * @return SYSTEM_OK on success, error code on failure
 */
system_result_t adc_read_battery_voltage(uint16_t *voltage_mv);

/**
 * @brief Read internal temperature sensor
 * @param temperature_c Pointer to store temperature in Celsius
 * @return SYSTEM_OK on success, error code on failure
 */
system_result_t adc_read_internal_temperature(float *temperature_c);

/**
 * @brief Read light sensor value
 * @param light_percentage Pointer to store light level as percentage (0-100)
 * @return SYSTEM_OK on success, error code on failure
 */
system_result_t adc_read_light_sensor(uint8_t *light_percentage);

/**
 * @brief Read external temperature sensor
 * @param temperature_c Pointer to store temperature in Celsius
 * @return SYSTEM_OK on success, error code on failure
 */
system_result_t adc_read_external_temperature(float *temperature_c);

/**
 * @brief Start continuous ADC sampling on a channel
 * @param channel ADC channel to sample
 * @param callback Function to call with new readings
 * @return SYSTEM_OK on success, error code on failure
 */
system_result_t adc_start_continuous(adc_channel_t channel, void (*callback)(adc_reading_t *reading));

/**
 * @brief Stop continuous ADC sampling
 * @param channel ADC channel to stop
 * @return SYSTEM_OK on success, error code on failure
 */
system_result_t adc_stop_continuous(adc_channel_t channel);

/**
 * @brief Calibrate ADC channel
 * @param channel ADC channel to calibrate
 * @param reference_voltage Known reference voltage for calibration
 * @return SYSTEM_OK on success, error code on failure
 */
system_result_t adc_calibrate_channel(adc_channel_t channel, float reference_voltage);

/**
 * @brief Get ADC channel name
 * @param channel ADC channel
 * @return Channel name string
 */
const char* adc_get_channel_name(adc_channel_t channel);

/**
 * @brief Update all ADC readings (call regularly from main loop)
 * @return SYSTEM_OK on success, error code on failure
 */
system_result_t adc_update_all(void);

#endif // ADC_DRIVER_H