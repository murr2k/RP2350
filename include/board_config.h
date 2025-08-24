#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include "pico/stdlib.h"

// RP2350-LCD-1.28 Board Pin Definitions

// SPI Display (GC9A01A)
#define DISPLAY_SPI_PORT spi1
#define DISPLAY_SCK_PIN  10
#define DISPLAY_MOSI_PIN 11
#define DISPLAY_CS_PIN   9
#define DISPLAY_DC_PIN   8
#define DISPLAY_RST_PIN  12
#define DISPLAY_BL_PIN   25  // PWM backlight control

// Display specifications
#define DISPLAY_WIDTH    240
#define DISPLAY_HEIGHT   240
#define DISPLAY_ROTATION 0

// I2C Bus (QMI8658C IMU)
#define I2C_PORT         i2c1
#define I2C_SDA_PIN      6
#define I2C_SCL_PIN      7
#define I2C_FREQUENCY    400000  // 400kHz

// IMU Sensor (QMI8658C)
#define IMU_I2C_ADDR     0x6B
#define IMU_INT1_PIN     22
#define IMU_INT2_PIN     24

// Buttons
#define BUTTON_A_PIN     15
#define BUTTON_B_PIN     17
#define BUTTON_X_PIN     19  
#define BUTTON_Y_PIN     21

// LEDs (if available)
#define LED_PIN          PICO_DEFAULT_LED_PIN

// ADC Pins (for future expansion)
#define ADC_VBAT_PIN     26  // ADC0
#define ADC_TEMP_PIN     27  // ADC1
#define ADC_LIGHT_PIN    28  // ADC2

// Power management
#define POWER_EN_PIN     23

// System configuration
#define SYSTEM_CLOCK_KHZ 150000  // 150MHz
#define WATCHDOG_TIMEOUT_MS 8300

// USB Serial configuration
#define USB_VENDOR_ID    0x2E8A  // Raspberry Pi Foundation
#define USB_PRODUCT_ID   0x000A  // Pico
#define USB_DEVICE_VER   0x0100
#define USB_MANUFACTURER "Waveshare"
#define USB_PRODUCT      "RP2350-LCD-1.28"

// Debug configuration
#ifdef DEBUG
    #define DEBUG_PRINT(fmt, ...) printf("[DEBUG] " fmt "\n", ##__VA_ARGS__)
#else
    #define DEBUG_PRINT(fmt, ...) ((void)0)
#endif

// Error handling macros
#define ASSERT(condition) \
    do { \
        if (!(condition)) { \
            printf("ASSERT FAILED: %s:%d - %s\n", __FILE__, __LINE__, #condition); \
            while(1) tight_loop_contents(); \
        } \
    } while(0)

#define CHECK_RESULT(result, expected) \
    do { \
        if ((result) != (expected)) { \
            printf("ERROR: %s:%d - Expected %d, got %d\n", __FILE__, __LINE__, (expected), (result)); \
            return (result); \
        } \
    } while(0)

#endif // BOARD_CONFIG_H