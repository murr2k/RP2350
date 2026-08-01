/**
 * TCA9554PWR IO expander.
 *
 * The RP2350 board wired LCD reset, LCD chip select and the touch reset
 * straight to GPIOs. On this board all three sit behind this expander, so it
 * has to come up before the display does.
 *
 * Pins are numbered 1..8 to match Waveshare's EXIO naming: pin N is bit N-1.
 */

#ifndef TCA9554_H
#define TCA9554_H

#include <stdint.h>

#include "esp_err.h"

#define TCA9554_REG_INPUT    0x00
#define TCA9554_REG_OUTPUT   0x01
#define TCA9554_REG_POLARITY 0x02
#define TCA9554_REG_CONFIG   0x03

/** Configure pin directions. Bit set = input, bit clear = output.
 *  Pass 0x00 to make every pin an output. */
esp_err_t TCA9554_Init(uint8_t direction_mask);

/** Drive one pin (1..8) high or low, leaving the others alone. */
esp_err_t TCA9554_Set(uint8_t pin, uint8_t level);

/** Read the level of one pin (1..8). */
uint8_t TCA9554_Get(uint8_t pin);

/** Drive all eight outputs at once. */
esp_err_t TCA9554_SetAll(uint8_t levels);

/** Read a register directly. */
uint8_t TCA9554_ReadReg(uint8_t reg);

#endif /* TCA9554_H */
