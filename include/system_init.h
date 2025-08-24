#ifndef SYSTEM_INIT_H
#define SYSTEM_INIT_H

#include "system_types.h"

/**
 * @brief Initialize all system components
 * 
 * This function initializes all hardware peripherals and drivers:
 * - GPIO pins
 * - SPI for display
 * - I2C for IMU
 * - ADC for sensors
 * - PWM for backlight
 * - Display driver
 * - IMU driver  
 * - Button driver
 * - ADC driver
 * 
 * @return SYSTEM_OK on success, error code on failure
 */
system_result_t system_init(void);

#endif // SYSTEM_INIT_H