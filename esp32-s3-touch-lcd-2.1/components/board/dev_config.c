#include "dev_config.h"

#include <string.h>

#include "board_config.h"
#include "driver/ledc.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tca9554.h"

static const char *TAG = "board";

#define I2C_TIMEOUT_MS 100
#define I2C_MAX_DEVICES 8

static i2c_master_bus_handle_t s_bus;

static struct {
    uint8_t addr;
    i2c_master_dev_handle_t handle;
} s_devices[I2C_MAX_DEVICES];
static int s_device_count;

static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t s_adc_cali;
static uint8_t s_backlight = 100;
static bool s_initialised;

/* --- timing --------------------------------------------------------------- */

void DEV_Delay_ms(uint32_t ms)
{
    if (ms == 0) {
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(ms) ? pdMS_TO_TICKS(ms) : 1);
}

void DEV_Delay_us(uint32_t us)
{
    esp_rom_delay_us(us);
}

uint64_t DEV_Time_us(void)
{
    return (uint64_t)esp_timer_get_time();
}

/* --- I2C ------------------------------------------------------------------ */

i2c_master_bus_handle_t DEV_I2C_Bus(void)
{
    return s_bus;
}

esp_err_t DEV_I2C_Device(uint8_t addr, i2c_master_dev_handle_t *out)
{
    if (s_bus == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    for (int i = 0; i < s_device_count; i++) {
        if (s_devices[i].addr == addr) {
            *out = s_devices[i].handle;
            return ESP_OK;
        }
    }
    if (s_device_count >= I2C_MAX_DEVICES) {
        return ESP_ERR_NO_MEM;
    }

    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = BOARD_I2C_HZ,
    };
    i2c_master_dev_handle_t handle;
    esp_err_t err = i2c_master_bus_add_device(s_bus, &cfg, &handle);
    if (err != ESP_OK) {
        return err;
    }
    s_devices[s_device_count].addr = addr;
    s_devices[s_device_count].handle = handle;
    s_device_count++;
    *out = handle;
    return ESP_OK;
}

bool DEV_I2C_Probe(uint8_t addr)
{
    return s_bus != NULL && i2c_master_probe(s_bus, addr, I2C_TIMEOUT_MS) == ESP_OK;
}

esp_err_t DEV_I2C_Write_nByte(uint8_t addr, const uint8_t *data, size_t len)
{
    i2c_master_dev_handle_t dev;
    esp_err_t err = DEV_I2C_Device(addr, &dev);
    if (err != ESP_OK) {
        return err;
    }
    return i2c_master_transmit(dev, data, len, I2C_TIMEOUT_MS);
}

esp_err_t DEV_I2C_Write_Byte(uint8_t addr, uint8_t reg, uint8_t value)
{
    const uint8_t buf[2] = {reg, value};
    return DEV_I2C_Write_nByte(addr, buf, sizeof(buf));
}

esp_err_t DEV_I2C_Read_nByte(uint8_t addr, uint8_t reg, uint8_t *data, size_t len)
{
    i2c_master_dev_handle_t dev;
    esp_err_t err = DEV_I2C_Device(addr, &dev);
    if (err != ESP_OK) {
        return err;
    }
    return i2c_master_transmit_receive(dev, &reg, 1, data, len, I2C_TIMEOUT_MS);
}

uint8_t DEV_I2C_Read_Byte(uint8_t addr, uint8_t reg)
{
    uint8_t value = 0;
    esp_err_t err = DEV_I2C_Read_nByte(addr, reg, &value, 1);
    if (err != ESP_OK) {
        ESP_LOGD(TAG, "read 0x%02x reg 0x%02x failed: %s", addr, reg, esp_err_to_name(err));
        return 0;
    }
    return value;
}

/* --- backlight ------------------------------------------------------------ */

static esp_err_t backlight_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = BOARD_LCD_BL_RES_BITS,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = BOARD_LCD_BL_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&timer);
    if (err != ESP_OK) {
        return err;
    }

    ledc_channel_config_t channel = {
        .gpio_num = BOARD_LCD_BL_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };
    err = ledc_channel_config(&channel);
    if (err != ESP_OK) {
        return err;
    }
    DEV_SET_PWM(s_backlight);
    return ESP_OK;
}

void DEV_SET_PWM(uint8_t percent)
{
    if (percent > 100) {
        percent = 100;
    }
    s_backlight = percent;

    const uint32_t max_duty = (1u << BOARD_LCD_BL_RES_BITS) - 1u;
    const uint32_t duty = (uint32_t)((max_duty * (uint32_t)percent) / 100u);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

uint8_t DEV_GET_PWM(void)
{
    return s_backlight;
}

/* --- battery -------------------------------------------------------------- */

static esp_err_t battery_init(void)
{
    adc_oneshot_unit_init_cfg_t unit = {
        .unit_id = ADC_UNIT_1,
    };
    esp_err_t err = adc_oneshot_new_unit(&unit, &s_adc);
    if (err != ESP_OK) {
        return err;
    }

    adc_oneshot_chan_cfg_t channel = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    err = adc_oneshot_config_channel(s_adc, BOARD_BAT_ADC_CHANNEL, &channel);
    if (err != ESP_OK) {
        return err;
    }

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali = {
        .unit_id = ADC_UNIT_1,
        .chan = BOARD_BAT_ADC_CHANNEL,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali, &s_adc_cali) != ESP_OK) {
        s_adc_cali = NULL;
        ESP_LOGW(TAG, "no ADC calibration data, battery reading is approximate");
    }
#endif
    return ESP_OK;
}

uint16_t DEV_ADC_Read(void)
{
    int raw = 0;
    if (s_adc == NULL || adc_oneshot_read(s_adc, BOARD_BAT_ADC_CHANNEL, &raw) != ESP_OK) {
        return 0;
    }
    return (uint16_t)raw;
}

float DEV_Battery_Volts(void)
{
    int raw = 0;
    if (s_adc == NULL || adc_oneshot_read(s_adc, BOARD_BAT_ADC_CHANNEL, &raw) != ESP_OK) {
        return 0.0f;
    }

    int millivolts;
    if (s_adc_cali == NULL || adc_cali_raw_to_voltage(s_adc_cali, raw, &millivolts) != ESP_OK) {
        /* 12 dB attenuation puts full scale near 3.1 V on a 12-bit reading. */
        millivolts = (int)((float)raw * 3100.0f / 4095.0f);
    }
    return ((float)millivolts * BOARD_BAT_DIVIDER / 1000.0f) / BOARD_BAT_CALIBRATION;
}

/* --- init ----------------------------------------------------------------- */

uint8_t DEV_Module_Init(void)
{
    if (s_initialised) {
        return 0;
    }

    i2c_master_bus_config_t bus = {
        .i2c_port = BOARD_I2C_PORT,
        .sda_io_num = BOARD_I2C_SDA_GPIO,
        .scl_io_num = BOARD_I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t err = i2c_new_master_bus(&bus, &s_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed: %s", esp_err_to_name(err));
        return 1;
    }

    /* All expander pins are outputs, and everything it drives is active low,
     * so park them high before anything else touches the panel or the touch
     * controller. */
    err = TCA9554_Init(0x00);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "IO expander not responding: %s", esp_err_to_name(err));
        return 2;
    }
    TCA9554_SetAll(0xFF);

    err = backlight_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "backlight init failed: %s", esp_err_to_name(err));
        return 3;
    }

    err = battery_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "battery ADC unavailable: %s", esp_err_to_name(err));
    }

    s_initialised = true;
    return 0;
}

void DEV_Module_Exit(void)
{
    DEV_SET_PWM(0);
}
