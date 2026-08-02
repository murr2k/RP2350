#include "cst820.h"

#include "board_config.h"
#include "dev_config.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "lcd_2in1.h"
#include "tca9554.h"

static const char *TAG = "cst820";

#define CST820_REG_GESTURE      0x01
#define CST820_REG_VERSION      0x15
#define CST820_REG_CHIP_ID      0xA7
#define CST820_REG_DIS_AUTOSLEEP 0xFE

static bool s_present;

static void touch_reset(void)
{
    TCA9554_Set(BOARD_EXIO_TOUCH_RST, 0);
    DEV_Delay_ms(10);
    TCA9554_Set(BOARD_EXIO_TOUCH_RST, 1);
    DEV_Delay_ms(60);
}

bool cst820_init(void)
{
    const gpio_config_t io = {
        .pin_bit_mask = 1ULL << BOARD_TOUCH_INT_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);

    touch_reset();

    /* Non-zero in this register keeps the controller awake. Polled reads stop
     * working once it naps, and the demos poll. */
    DEV_I2C_Write_Byte(BOARD_TOUCH_ADDR, CST820_REG_DIS_AUTOSLEEP, 0xFF);

    if (!DEV_I2C_Probe(BOARD_TOUCH_ADDR)) {
        ESP_LOGE(TAG, "no touch controller at 0x%02x", BOARD_TOUCH_ADDR);
        s_present = false;
        return false;
    }

    uint8_t id[3] = {0};
    DEV_I2C_Read_nByte(BOARD_TOUCH_ADDR, CST820_REG_CHIP_ID, id, sizeof(id));
    const uint8_t version = DEV_I2C_Read_Byte(BOARD_TOUCH_ADDR, CST820_REG_VERSION);
    ESP_LOGI(TAG, "ready: chip 0x%02x proj 0x%02x fw 0x%02x version 0x%02x",
             id[0], id[1], id[2], version);

    s_present = true;
    return true;
}

bool cst820_present(void)
{
    return s_present;
}

bool cst820_read(touch_state_t *out)
{
    touch_state_t state = {0};

    if (!s_present) {
        if (out != NULL) {
            *out = state;
        }
        return false;
    }

    uint8_t buf[6] = {0};
    if (DEV_I2C_Read_nByte(BOARD_TOUCH_ADDR, CST820_REG_GESTURE, buf, sizeof(buf)) != ESP_OK) {
        if (out != NULL) {
            *out = state;
        }
        return false;
    }

    state.gesture = (touch_gesture_t)buf[0];
    state.contacts = buf[1];
    state.pressed = (buf[1] != 0);
    if (state.pressed) {
        state.x = (uint16_t)(((buf[2] & 0x0F) << 8) | buf[3]);
        state.y = (uint16_t)(((buf[4] & 0x0F) << 8) | buf[5]);
        if (state.x >= LCD_2IN1_WIDTH) {
            state.x = LCD_2IN1_WIDTH - 1;
        }
        if (state.y >= LCD_2IN1_HEIGHT) {
            state.y = LCD_2IN1_HEIGHT - 1;
        }
    }

    if (out != NULL) {
        *out = state;
    }
    return state.pressed;
}

bool cst820_read_raw(uint8_t *out, int len)
{
    if (!s_present || out == NULL || len <= 0) {
        return false;
    }
    return DEV_I2C_Read_nByte(BOARD_TOUCH_ADDR, 0x00, out, (uint32_t)len) == ESP_OK;
}

const char *cst820_gesture_name(touch_gesture_t gesture)
{
    switch (gesture) {
    case TOUCH_GESTURE_NONE:         return "none";
    case TOUCH_GESTURE_SWIPE_UP:     return "swipe up";
    case TOUCH_GESTURE_SWIPE_DOWN:   return "swipe down";
    case TOUCH_GESTURE_SWIPE_LEFT:   return "swipe left";
    case TOUCH_GESTURE_SWIPE_RIGHT:  return "swipe right";
    case TOUCH_GESTURE_SINGLE_CLICK: return "single click";
    case TOUCH_GESTURE_DOUBLE_CLICK: return "double click";
    case TOUCH_GESTURE_LONG_PRESS:   return "long press";
    default:                         return "unknown";
    }
}
