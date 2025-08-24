#include "system_init.h"
#include "board_config.h"
#include "system_types.h"
#include "display_driver.h"
#include "imu_driver.h"
#include "button_driver.h"
#include "adc_driver.h"

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"

extern system_state_t g_system_state;

static system_result_t init_gpio(void) {
    // Initialize GPIO pins
    
    // Button pins (with pull-up resistors)
    gpio_init(BUTTON_A_PIN);
    gpio_set_dir(BUTTON_A_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_A_PIN);
    
    gpio_init(BUTTON_B_PIN);
    gpio_set_dir(BUTTON_B_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_B_PIN);
    
    gpio_init(BUTTON_X_PIN);
    gpio_set_dir(BUTTON_X_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_X_PIN);
    
    gpio_init(BUTTON_Y_PIN);
    gpio_set_dir(BUTTON_Y_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_Y_PIN);
    
    // LED pin
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);
    
    // Power enable pin
    gpio_init(POWER_EN_PIN);
    gpio_set_dir(POWER_EN_PIN, GPIO_OUT);
    gpio_put(POWER_EN_PIN, 1);  // Enable power
    
    // IMU interrupt pins
    gpio_init(IMU_INT1_PIN);
    gpio_set_dir(IMU_INT1_PIN, GPIO_IN);
    gpio_pull_up(IMU_INT1_PIN);
    
    gpio_init(IMU_INT2_PIN);
    gpio_set_dir(IMU_INT2_PIN, GPIO_IN);
    gpio_pull_up(IMU_INT2_PIN);
    
    return SYSTEM_OK;
}

static system_result_t init_spi(void) {
    // Initialize SPI for display
    spi_init(DISPLAY_SPI_PORT, 62500000);  // 62.5 MHz
    
    gpio_set_function(DISPLAY_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(DISPLAY_MOSI_PIN, GPIO_FUNC_SPI);
    
    // SPI format: 8 bits, polarity 0, phase 0, MSB first
    spi_set_format(DISPLAY_SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    
    DEBUG_PRINT("SPI initialized at %d Hz", spi_set_baudrate(DISPLAY_SPI_PORT, 62500000));
    
    return SYSTEM_OK;
}

static system_result_t init_i2c(void) {
    // Initialize I2C for IMU
    i2c_init(I2C_PORT, I2C_FREQUENCY);
    
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    
    // Enable pull-up resistors
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    
    DEBUG_PRINT("I2C initialized at %d Hz", I2C_FREQUENCY);
    
    return SYSTEM_OK;
}

static system_result_t init_adc(void) {
    // Initialize ADC
    adc_init();
    
    // Initialize ADC pins
    adc_gpio_init(ADC_VBAT_PIN);
    adc_gpio_init(ADC_TEMP_PIN);
    adc_gpio_init(ADC_LIGHT_PIN);
    
    // Enable temperature sensor
    adc_set_temp_sensor_enabled(true);
    
    DEBUG_PRINT("ADC initialized");
    
    return SYSTEM_OK;
}

static system_result_t init_pwm(void) {
    // Initialize PWM for display backlight
    uint slice_num = pwm_gpio_to_slice_num(DISPLAY_BL_PIN);
    
    gpio_set_function(DISPLAY_BL_PIN, GPIO_FUNC_PWM);
    
    // Set PWM frequency to ~1kHz
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 125.0f);  // 125MHz / 125 = 1MHz
    pwm_config_set_wrap(&config, 999);       // 1MHz / 1000 = 1kHz
    
    pwm_init(slice_num, &config, true);
    
    // Set initial brightness to 50%
    pwm_set_gpio_level(DISPLAY_BL_PIN, 500);
    g_system_state.brightness = 128;  // 50% of 255
    
    DEBUG_PRINT("PWM initialized for backlight control");
    
    return SYSTEM_OK;
}

system_result_t system_init(void) {
    system_result_t result;
    
    DEBUG_PRINT("Initializing GPIO...");
    result = init_gpio();
    if (result != SYSTEM_OK) {
        printf("GPIO initialization failed: %d\n", result);
        return result;
    }
    
    DEBUG_PRINT("Initializing SPI...");
    result = init_spi();
    if (result != SYSTEM_OK) {
        printf("SPI initialization failed: %d\n", result);
        return result;
    }
    
    DEBUG_PRINT("Initializing I2C...");
    result = init_i2c();
    if (result != SYSTEM_OK) {
        printf("I2C initialization failed: %d\n", result);
        return result;
    }
    
    DEBUG_PRINT("Initializing ADC...");
    result = init_adc();
    if (result != SYSTEM_OK) {
        printf("ADC initialization failed: %d\n", result);
        return result;
    }
    
    DEBUG_PRINT("Initializing PWM...");
    result = init_pwm();
    if (result != SYSTEM_OK) {
        printf("PWM initialization failed: %d\n", result);
        return result;
    }
    
    DEBUG_PRINT("Initializing display driver...");
    result = display_init();
    if (result != SYSTEM_OK) {
        printf("Display initialization failed: %d\n", result);
        return result;
    }
    
    DEBUG_PRINT("Initializing IMU driver...");
    result = imu_init();
    if (result != SYSTEM_OK) {
        printf("IMU initialization failed: %d\n", result);
        return result;
    }
    
    DEBUG_PRINT("Initializing button driver...");
    result = button_init();
    if (result != SYSTEM_OK) {
        printf("Button initialization failed: %d\n", result);
        return result;
    }
    
    DEBUG_PRINT("Initializing ADC driver...");
    result = adc_driver_init();
    if (result != SYSTEM_OK) {
        printf("ADC driver initialization failed: %d\n", result);
        return result;
    }
    
    // Flash LED to indicate successful initialization
    for (int i = 0; i < 3; i++) {
        gpio_put(LED_PIN, 1);
        sleep_ms(100);
        gpio_put(LED_PIN, 0);
        sleep_ms(100);
    }
    
    printf("System initialization complete\n");
    return SYSTEM_OK;
}