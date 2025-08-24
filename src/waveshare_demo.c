/**
 * Waveshare RP2350-LCD-1.28 Demo
 * Based on official Waveshare example configuration
 */

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

// LCD Pin definitions (Waveshare standard)
#define LCD_DC_PIN      8
#define LCD_CS_PIN      9
#define LCD_SCLK_PIN    10
#define LCD_MOSI_PIN    11
#define LCD_RST_PIN     12
#define LCD_BL_PIN      25

// IMU I2C pins
#define IMU_SDA_PIN     6
#define IMU_SCL_PIN     7
#define QMI8658_ADDR    0x6B

// Buttons
#define BUTTON_A_PIN    15
#define BUTTON_B_PIN    17

// Battery ADC
#define BAT_ADC_PIN     29

// GC9A01 Commands
#define GC9A01_SWRESET  0x01
#define GC9A01_SLPOUT   0x11
#define GC9A01_DISPON   0x29
#define GC9A01_CASET    0x2A
#define GC9A01_RASET    0x2B
#define GC9A01_RAMWR    0x2C
#define GC9A01_MADCTL   0x36
#define GC9A01_COLMOD   0x3A
#define GC9A01_INVON    0x21

// QMI8658 Registers
#define QMI8658_WHO_AM_I    0x00
#define QMI8658_CTRL1       0x02
#define QMI8658_CTRL2       0x03
#define QMI8658_CTRL3       0x04
#define QMI8658_CTRL7       0x08
#define QMI8658_AX_L        0x35
#define QMI8658_TEMP_L      0x33

// Simple 5x7 font for text display
static const uint8_t font5x7[][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, // Space
    {0x00, 0x00, 0x5F, 0x00, 0x00}, // !
    {0x00, 0x07, 0x00, 0x07, 0x00}, // "
    {0x14, 0x7F, 0x14, 0x7F, 0x14}, // #
    // ... (abbreviated for space, full font would be included)
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, // 0
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // 1
    {0x42, 0x61, 0x51, 0x49, 0x46}, // 2
    {0x21, 0x41, 0x45, 0x4B, 0x31}, // 3
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // 4
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 6
    {0x01, 0x71, 0x09, 0x05, 0x03}, // 7
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 8
    {0x06, 0x49, 0x49, 0x29, 0x1E}, // 9
};

// IMU data structure
typedef struct {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t temp;
} imu_data_t;

static imu_data_t imu_data;
static float battery_voltage = 0.0f;

// LCD functions
static void lcd_write_cmd(uint8_t cmd) {
    gpio_put(LCD_CS_PIN, 0);
    gpio_put(LCD_DC_PIN, 0);
    spi_write_blocking(spi1, &cmd, 1);
    gpio_put(LCD_CS_PIN, 1);
}

static void lcd_write_data(uint8_t data) {
    gpio_put(LCD_CS_PIN, 0);
    gpio_put(LCD_DC_PIN, 1);
    spi_write_blocking(spi1, &data, 1);
    gpio_put(LCD_CS_PIN, 1);
}

static void lcd_write_data_bulk(const uint8_t *data, size_t len) {
    gpio_put(LCD_CS_PIN, 0);
    gpio_put(LCD_DC_PIN, 1);
    spi_write_blocking(spi1, data, len);
    gpio_put(LCD_CS_PIN, 1);
}

static void lcd_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    lcd_write_cmd(GC9A01_CASET);
    lcd_write_data(x0 >> 8);
    lcd_write_data(x0 & 0xFF);
    lcd_write_data(x1 >> 8);
    lcd_write_data(x1 & 0xFF);
    
    lcd_write_cmd(GC9A01_RASET);
    lcd_write_data(y0 >> 8);
    lcd_write_data(y0 & 0xFF);
    lcd_write_data(y1 >> 8);
    lcd_write_data(y1 & 0xFF);
    
    lcd_write_cmd(GC9A01_RAMWR);
}

static void lcd_init(void) {
    // Initialize control pins
    gpio_init(LCD_DC_PIN);
    gpio_set_dir(LCD_DC_PIN, GPIO_OUT);
    
    gpio_init(LCD_CS_PIN);
    gpio_set_dir(LCD_CS_PIN, GPIO_OUT);
    gpio_put(LCD_CS_PIN, 1);
    
    gpio_init(LCD_RST_PIN);
    gpio_set_dir(LCD_RST_PIN, GPIO_OUT);
    
    // Initialize SPI
    spi_init(spi1, 62500000);  // 62.5MHz
    gpio_set_function(LCD_SCLK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(LCD_MOSI_PIN, GPIO_FUNC_SPI);
    
    // Initialize backlight
    gpio_init(LCD_BL_PIN);
    gpio_set_dir(LCD_BL_PIN, GPIO_OUT);
    gpio_put(LCD_BL_PIN, 1);
    
    // Hardware reset
    gpio_put(LCD_RST_PIN, 1);
    sleep_ms(5);
    gpio_put(LCD_RST_PIN, 0);
    sleep_ms(10);
    gpio_put(LCD_RST_PIN, 1);
    sleep_ms(120);
    
    // Software reset
    lcd_write_cmd(0x01);
    sleep_ms(120);
    
    // Sleep out
    lcd_write_cmd(0x11);
    sleep_ms(120);
    
    // Interface Pixel Format
    lcd_write_cmd(0x3A);
    lcd_write_data(0x55);  // 16-bit RGB565
    
    // Memory Access Control
    lcd_write_cmd(0x36);
    lcd_write_data(0x00);
    
    // Display Inversion On
    lcd_write_cmd(0x21);
    
    // Display On
    lcd_write_cmd(0x29);
    
    // Set backlight PWM
    gpio_set_function(LCD_BL_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(LCD_BL_PIN);
    pwm_set_wrap(slice, 255);
    pwm_set_gpio_level(LCD_BL_PIN, 200);  // ~80% brightness
    pwm_set_enabled(slice, true);
}

static void lcd_clear(uint16_t color) {
    lcd_set_window(0, 0, 239, 239);
    
    uint8_t data[2];
    data[0] = color >> 8;
    data[1] = color & 0xFF;
    
    gpio_put(LCD_CS_PIN, 0);
    gpio_put(LCD_DC_PIN, 1);
    
    for (int i = 0; i < 240 * 240; i++) {
        spi_write_blocking(spi1, data, 2);
    }
    
    gpio_put(LCD_CS_PIN, 1);
}

static void lcd_draw_pixel(uint16_t x, uint16_t y, uint16_t color) {
    if (x >= 240 || y >= 240) return;
    
    lcd_set_window(x, y, x, y);
    
    uint8_t data[2];
    data[0] = color >> 8;
    data[1] = color & 0xFF;
    lcd_write_data_bulk(data, 2);
}

static void lcd_draw_char(uint16_t x, uint16_t y, char c, uint16_t color, uint16_t bg, uint8_t size) {
    if (c < ' ' || c > '9') return;
    
    int idx = (c == ' ') ? 0 : (c >= '0' && c <= '9') ? (c - '0' + 3) : 0;
    
    for (int i = 0; i < 5; i++) {
        uint8_t line = font5x7[idx][i];
        for (int j = 0; j < 7; j++) {
            if (line & (1 << j)) {
                if (size == 1) {
                    lcd_draw_pixel(x + i, y + j, color);
                } else {
                    for (int sx = 0; sx < size; sx++) {
                        for (int sy = 0; sy < size; sy++) {
                            lcd_draw_pixel(x + i*size + sx, y + j*size + sy, color);
                        }
                    }
                }
            } else if (bg != color) {
                if (size == 1) {
                    lcd_draw_pixel(x + i, y + j, bg);
                } else {
                    for (int sx = 0; sx < size; sx++) {
                        for (int sy = 0; sy < size; sy++) {
                            lcd_draw_pixel(x + i*size + sx, y + j*size + sy, bg);
                        }
                    }
                }
            }
        }
    }
}

static void lcd_draw_string(uint16_t x, uint16_t y, const char *str, uint16_t color, uint16_t bg, uint8_t size) {
    while (*str) {
        lcd_draw_char(x, y, *str, color, bg, size);
        x += 6 * size;
        str++;
    }
}

// IMU functions
static void imu_init(void) {
    i2c_init(i2c0, 400 * 1000);  // 400kHz
    gpio_set_function(IMU_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(IMU_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(IMU_SDA_PIN);
    gpio_pull_up(IMU_SCL_PIN);
    
    sleep_ms(10);
    
    // Check WHO_AM_I
    uint8_t reg = QMI8658_WHO_AM_I;
    uint8_t who_am_i;
    i2c_write_blocking(i2c0, QMI8658_ADDR, &reg, 1, true);
    i2c_read_blocking(i2c0, QMI8658_ADDR, &who_am_i, 1, false);
    
    printf("QMI8658 WHO_AM_I: 0x%02X\n", who_am_i);
    
    // Configure QMI8658
    uint8_t config[2];
    
    // Enable accelerometer and gyroscope
    config[0] = QMI8658_CTRL1;
    config[1] = 0x60;  // ACC: ±8g, GYRO: ±512dps
    i2c_write_blocking(i2c0, QMI8658_ADDR, config, 2, false);
    
    // Set ACC ODR to 250Hz
    config[0] = QMI8658_CTRL2;
    config[1] = 0x05;
    i2c_write_blocking(i2c0, QMI8658_ADDR, config, 2, false);
    
    // Set GYRO ODR to 250Hz
    config[0] = QMI8658_CTRL3;
    config[1] = 0x50;
    i2c_write_blocking(i2c0, QMI8658_ADDR, config, 2, false);
    
    // Enable sensors
    config[0] = QMI8658_CTRL7;
    config[1] = 0x03;
    i2c_write_blocking(i2c0, QMI8658_ADDR, config, 2, false);
}

static void imu_read(void) {
    uint8_t reg = QMI8658_AX_L;
    uint8_t buffer[14];  // 6 bytes ACC + 6 bytes GYRO + 2 bytes TEMP
    
    i2c_write_blocking(i2c0, QMI8658_ADDR, &reg, 1, true);
    i2c_read_blocking(i2c0, QMI8658_ADDR, buffer, 14, false);
    
    imu_data.ax = (int16_t)(buffer[1] << 8 | buffer[0]);
    imu_data.ay = (int16_t)(buffer[3] << 8 | buffer[2]);
    imu_data.az = (int16_t)(buffer[5] << 8 | buffer[4]);
    imu_data.gx = (int16_t)(buffer[7] << 8 | buffer[6]);
    imu_data.gy = (int16_t)(buffer[9] << 8 | buffer[8]);
    imu_data.gz = (int16_t)(buffer[11] << 8 | buffer[10]);
    
    // Read temperature
    reg = QMI8658_TEMP_L;
    uint8_t temp_buffer[2];
    i2c_write_blocking(i2c0, QMI8658_ADDR, &reg, 1, true);
    i2c_read_blocking(i2c0, QMI8658_ADDR, temp_buffer, 2, false);
    imu_data.temp = (int16_t)(temp_buffer[1] << 8 | temp_buffer[0]);
}

static void display_imu_data(void) {
    char buffer[32];
    
    // Title
    lcd_draw_string(70, 20, "RP2350 IMU Demo", 0xFFFF, 0x0000, 2);
    
    // Accelerometer data
    lcd_draw_string(20, 60, "Accelerometer:", 0x07E0, 0x0000, 1);
    
    sprintf(buffer, "AX: %6d", imu_data.ax);
    lcd_draw_string(20, 75, buffer, 0xFFFF, 0x0000, 1);
    
    sprintf(buffer, "AY: %6d", imu_data.ay);
    lcd_draw_string(20, 90, buffer, 0xFFFF, 0x0000, 1);
    
    sprintf(buffer, "AZ: %6d", imu_data.az);
    lcd_draw_string(20, 105, buffer, 0xFFFF, 0x0000, 1);
    
    // Gyroscope data
    lcd_draw_string(20, 125, "Gyroscope:", 0x07FF, 0x0000, 1);
    
    sprintf(buffer, "GX: %6d", imu_data.gx);
    lcd_draw_string(20, 140, buffer, 0xFFFF, 0x0000, 1);
    
    sprintf(buffer, "GY: %6d", imu_data.gy);
    lcd_draw_string(20, 155, buffer, 0xFFFF, 0x0000, 1);
    
    sprintf(buffer, "GZ: %6d", imu_data.gz);
    lcd_draw_string(20, 170, buffer, 0xFFFF, 0x0000, 1);
    
    // Temperature
    float temp_c = imu_data.temp / 256.0f + 25.0f;
    sprintf(buffer, "Temp: %.1f C", temp_c);
    lcd_draw_string(20, 190, buffer, 0xFFE0, 0x0000, 1);
    
    // Battery voltage
    sprintf(buffer, "Battery: %.2f V", battery_voltage);
    lcd_draw_string(20, 210, buffer, 0xF800, 0x0000, 1);
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    
    printf("\n=== Waveshare RP2350-LCD-1.28 Demo ===\n");
    
    // Initialize ADC for battery monitoring
    adc_init();
    adc_gpio_init(BAT_ADC_PIN);
    adc_select_input(3);  // GPIO29 is ADC3
    
    // Initialize buttons
    gpio_init(BUTTON_A_PIN);
    gpio_set_dir(BUTTON_A_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_A_PIN);
    
    gpio_init(BUTTON_B_PIN);
    gpio_set_dir(BUTTON_B_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_B_PIN);
    
    printf("Initializing LCD...\n");
    lcd_init();
    lcd_clear(0x0000);  // Black background
    
    printf("Initializing IMU...\n");
    imu_init();
    
    printf("Starting main loop...\n");
    
    while (1) {
        // Read IMU data
        imu_read();
        
        // Read battery voltage (with voltage divider factor)
        uint16_t adc_raw = adc_read();
        battery_voltage = adc_raw * 3.3f * 2.0f / 4095.0f;
        
        // Update display
        display_imu_data();
        
        // Check buttons
        if (!gpio_get(BUTTON_A_PIN)) {
            printf("Button A pressed\n");
            lcd_clear(0x0000);  // Clear screen
        }
        
        if (!gpio_get(BUTTON_B_PIN)) {
            printf("Button B pressed\n");
            // Could add calibration or other function here
        }
        
        sleep_ms(100);  // 10Hz update rate
    }
    
    return 0;
}