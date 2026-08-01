/**
 * DEV_Config work-alike for the ESP32-S3-Touch-LCD-2.1.
 *
 * Same job as lib/Config/DEV_Config.h in the RP2350 project: one place that
 * owns the shared buses, the backlight and the timing helpers, so the demo
 * code never talks to the SDK directly. The names are kept close to the
 * originals to make the two trees easy to diff.
 *
 * Difference worth knowing: the ESP-IDF I2C master driver is handle based, so
 * DEV_I2C_* look up (and cache) a device handle for the address you pass.
 */

#ifndef DEV_CONFIG_H
#define DEV_CONFIG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

#define UBYTE   uint8_t
#define UWORD   uint16_t
#define UDOUBLE uint32_t

/** Bring up I2C, the IO expander, the backlight and the battery ADC.
 *  @return 0 on success, non-zero on failure (matches the Pico version). */
uint8_t DEV_Module_Init(void);

void DEV_Module_Exit(void);

/* --- timing --------------------------------------------------------------- */

void DEV_Delay_ms(uint32_t ms);
void DEV_Delay_us(uint32_t us);

/** Microseconds since boot. Work-alike of the Pico's time_us_32(), widened to
 *  64 bits because esp_timer already gives us that for free. */
uint64_t DEV_Time_us(void);

/* --- backlight ------------------------------------------------------------ */

/** Set backlight brightness, 0..100 percent. */
void DEV_SET_PWM(uint8_t percent);
uint8_t DEV_GET_PWM(void);

/* --- battery -------------------------------------------------------------- */

uint16_t DEV_ADC_Read(void);      /**< raw ADC counts on the battery divider */
float    DEV_Battery_Volts(void); /**< battery voltage in volts */

/* --- I2C ------------------------------------------------------------------ */

i2c_master_bus_handle_t DEV_I2C_Bus(void);

/** Get (creating on first use) a device handle for a 7-bit address. */
esp_err_t DEV_I2C_Device(uint8_t addr, i2c_master_dev_handle_t *out);

/** True if a device acknowledges its address on the shared bus. */
bool DEV_I2C_Probe(uint8_t addr);

esp_err_t DEV_I2C_Write_Byte(uint8_t addr, uint8_t reg, uint8_t value);
esp_err_t DEV_I2C_Write_nByte(uint8_t addr, const uint8_t *data, size_t len);
esp_err_t DEV_I2C_Read_nByte(uint8_t addr, uint8_t reg, uint8_t *data, size_t len);

/** Single register read. Returns 0 and logs on bus errors. */
uint8_t DEV_I2C_Read_Byte(uint8_t addr, uint8_t reg);

#endif /* DEV_CONFIG_H */
