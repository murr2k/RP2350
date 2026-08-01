#include "tca9554.h"

#include "board_config.h"
#include "dev_config.h"
#include "esp_log.h"

static const char *TAG = "tca9554";

/* Shadow of the output register. Reading it back over I2C on every pin change
 * works, but a shadow keeps the display reset sequence free of extra bus
 * traffic and survives the case where a read glitches. */
static uint8_t s_output = 0xFF;

esp_err_t TCA9554_Init(uint8_t direction_mask)
{
    esp_err_t err = DEV_I2C_Write_Byte(BOARD_TCA9554_ADDR, TCA9554_REG_OUTPUT, s_output);
    if (err != ESP_OK) {
        return err;
    }
    return DEV_I2C_Write_Byte(BOARD_TCA9554_ADDR, TCA9554_REG_CONFIG, direction_mask);
}

esp_err_t TCA9554_Set(uint8_t pin, uint8_t level)
{
    if (pin < 1 || pin > 8) {
        return ESP_ERR_INVALID_ARG;
    }
    const uint8_t bit = (uint8_t)(1u << (pin - 1));
    if (level) {
        s_output |= bit;
    } else {
        s_output &= (uint8_t)~bit;
    }

    esp_err_t err = DEV_I2C_Write_Byte(BOARD_TCA9554_ADDR, TCA9554_REG_OUTPUT, s_output);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "set EXIO%u failed: %s", pin, esp_err_to_name(err));
    }
    return err;
}

uint8_t TCA9554_Get(uint8_t pin)
{
    if (pin < 1 || pin > 8) {
        return 0;
    }
    const uint8_t input = DEV_I2C_Read_Byte(BOARD_TCA9554_ADDR, TCA9554_REG_INPUT);
    return (uint8_t)((input >> (pin - 1)) & 0x01u);
}

esp_err_t TCA9554_SetAll(uint8_t levels)
{
    s_output = levels;
    return DEV_I2C_Write_Byte(BOARD_TCA9554_ADDR, TCA9554_REG_OUTPUT, s_output);
}

uint8_t TCA9554_ReadReg(uint8_t reg)
{
    return DEV_I2C_Read_Byte(BOARD_TCA9554_ADDR, reg);
}
