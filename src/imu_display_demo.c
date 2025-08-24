/**
 * RP2350-LCD-1.28 IMU Display Demo
 * Displays 6-axis IMU data and battery voltage
 * Based on Waveshare official initialization
 */

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

// LCD Pin definitions
#define LCD_DC_PIN      8
#define LCD_CS_PIN      9
#define LCD_CLK_PIN     10
#define LCD_MOSI_PIN    11
#define LCD_RST_PIN     12
#define LCD_BL_PIN      25

// I2C pins for IMU
#define I2C_SDA_PIN     6
#define I2C_SCL_PIN     7

// Battery ADC
#define BAT_ADC_PIN     29
#define BAT_ADC_CHANNEL 3

// QMI8658 I2C address
#define QMI8658_ADDR    0x6B

// QMI8658 Register definitions
#define QMI8658_WHO_AM_I        0x00
#define QMI8658_REVISION_ID     0x01
#define QMI8658_CTRL1           0x02
#define QMI8658_CTRL2           0x03
#define QMI8658_CTRL3           0x04
#define QMI8658_CTRL4           0x05
#define QMI8658_CTRL5           0x06
#define QMI8658_CTRL6           0x07
#define QMI8658_CTRL7           0x08
#define QMI8658_CTRL8           0x09
#define QMI8658_CTRL9           0x0A
#define QMI8658_TEMP_L          0x33
#define QMI8658_TEMP_H          0x34
#define QMI8658_AX_L            0x35
#define QMI8658_AX_H            0x36
#define QMI8658_AY_L            0x37
#define QMI8658_AY_H            0x38
#define QMI8658_AZ_L            0x39
#define QMI8658_AZ_H            0x3A
#define QMI8658_GX_L            0x3B
#define QMI8658_GX_H            0x3C
#define QMI8658_GY_L            0x3D
#define QMI8658_GY_H            0x3E
#define QMI8658_GZ_L            0x3F
#define QMI8658_GZ_H            0x40

// Display dimensions
#define LCD_WIDTH  240
#define LCD_HEIGHT 240

// Font data (5x7 font)
static const uint8_t font5x7[][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, // Space
    {0x00, 0x00, 0x5F, 0x00, 0x00}, // !
    {0x00, 0x07, 0x00, 0x07, 0x00}, // "
    {0x14, 0x7F, 0x14, 0x7F, 0x14}, // #
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, // $
    {0x23, 0x13, 0x08, 0x64, 0x62}, // %
    {0x36, 0x49, 0x55, 0x22, 0x50}, // &
    {0x00, 0x05, 0x03, 0x00, 0x00}, // '
    {0x00, 0x1C, 0x22, 0x41, 0x00}, // (
    {0x00, 0x41, 0x22, 0x1C, 0x00}, // )
    {0x14, 0x08, 0x3E, 0x08, 0x14}, // *
    {0x08, 0x08, 0x3E, 0x08, 0x08}, // +
    {0x00, 0x50, 0x30, 0x00, 0x00}, // ,
    {0x08, 0x08, 0x08, 0x08, 0x08}, // -
    {0x00, 0x60, 0x60, 0x00, 0x00}, // .
    {0x20, 0x10, 0x08, 0x04, 0x02}, // /
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 0
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // 1
    {0x42, 0x61, 0x51, 0x49, 0x46}, // 2
    {0x21, 0x41, 0x45, 0x4B, 0x31}, // 3
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // 4
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 6
    {0x01, 0x71, 0x09, 0x05, 0x03}, // 7
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 8
    {0x06, 0x49, 0x49, 0x29, 0x1E}, // 9
    {0x00, 0x36, 0x36, 0x00, 0x00}, // :
    {0x00, 0x56, 0x36, 0x00, 0x00}, // ;
    {0x08, 0x14, 0x22, 0x41, 0x00}, // <
    {0x14, 0x14, 0x14, 0x14, 0x14}, // =
    {0x00, 0x41, 0x22, 0x14, 0x08}, // >
    {0x02, 0x01, 0x51, 0x09, 0x06}, // ?
    {0x32, 0x49, 0x79, 0x41, 0x3E}, // @
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, // A
    {0x7F, 0x49, 0x49, 0x49, 0x36}, // B
    {0x3E, 0x41, 0x41, 0x41, 0x22}, // C
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, // D
    {0x7F, 0x49, 0x49, 0x49, 0x41}, // E
    {0x7F, 0x09, 0x09, 0x09, 0x01}, // F
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, // G
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, // H
    {0x00, 0x41, 0x7F, 0x41, 0x00}, // I
    {0x20, 0x40, 0x41, 0x3F, 0x01}, // J
    {0x7F, 0x08, 0x14, 0x22, 0x41}, // K
    {0x7F, 0x40, 0x40, 0x40, 0x40}, // L
    {0x7F, 0x02, 0x0C, 0x02, 0x7F}, // M
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, // N
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, // O
    {0x7F, 0x09, 0x09, 0x09, 0x06}, // P
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, // Q
    {0x7F, 0x09, 0x19, 0x29, 0x46}, // R
    {0x46, 0x49, 0x49, 0x49, 0x31}, // S
    {0x01, 0x01, 0x7F, 0x01, 0x01}, // T
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, // U
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, // V
    {0x3F, 0x40, 0x38, 0x40, 0x3F}, // W
    {0x63, 0x14, 0x08, 0x14, 0x63}, // X
    {0x07, 0x08, 0x70, 0x08, 0x07}, // Y
    {0x61, 0x51, 0x49, 0x45, 0x43}, // Z
};

// IMU data structure
typedef struct {
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float temperature;
} imu_data_t;

// LCD functions
static void lcd_write_command(uint8_t cmd) {
    gpio_put(LCD_DC_PIN, 0);
    spi_write_blocking(spi1, &cmd, 1);
}

static void lcd_write_data(uint8_t data) {
    gpio_put(LCD_DC_PIN, 1);
    spi_write_blocking(spi1, &data, 1);
}

static void lcd_write_data16(uint16_t data) {
    uint8_t buf[2];
    buf[0] = data >> 8;
    buf[1] = data & 0xFF;
    gpio_put(LCD_DC_PIN, 1);
    spi_write_blocking(spi1, buf, 2);
}

static void lcd_reset(void) {
    gpio_put(LCD_RST_PIN, 1);
    sleep_ms(100);
    gpio_put(LCD_RST_PIN, 0);
    sleep_ms(100);
    gpio_put(LCD_RST_PIN, 1);
    gpio_put(LCD_CS_PIN, 0);  // Keep CS LOW
    sleep_ms(100);
}

static void lcd_init_reg(void) {
    // Waveshare initialization sequence
    lcd_write_command(0xEF);
    lcd_write_command(0xEB);
    lcd_write_data(0x14);
    
    lcd_write_command(0xFE);
    lcd_write_command(0xEF);
    
    lcd_write_command(0xEB);
    lcd_write_data(0x14);
    
    lcd_write_command(0x84);
    lcd_write_data(0x40);
    
    lcd_write_command(0x85);
    lcd_write_data(0xFF);
    
    lcd_write_command(0x86);
    lcd_write_data(0xFF);
    
    lcd_write_command(0x87);
    lcd_write_data(0xFF);
    
    lcd_write_command(0x88);
    lcd_write_data(0x0A);
    
    lcd_write_command(0x89);
    lcd_write_data(0x21);
    
    lcd_write_command(0x8A);
    lcd_write_data(0x00);
    
    lcd_write_command(0x8B);
    lcd_write_data(0x80);
    
    lcd_write_command(0x8C);
    lcd_write_data(0x01);
    
    lcd_write_command(0x8D);
    lcd_write_data(0x01);
    
    lcd_write_command(0x8E);
    lcd_write_data(0xFF);
    
    lcd_write_command(0x8F);
    lcd_write_data(0xFF);
    
    lcd_write_command(0xB6);
    lcd_write_data(0x00);
    lcd_write_data(0x20);
    
    lcd_write_command(0x36);
    lcd_write_data(0x08);
    
    lcd_write_command(0x3A);
    lcd_write_data(0x05);
    
    lcd_write_command(0x90);
    lcd_write_data(0x08);
    lcd_write_data(0x08);
    lcd_write_data(0x08);
    lcd_write_data(0x08);
    
    lcd_write_command(0xBD);
    lcd_write_data(0x06);
    
    lcd_write_command(0xBC);
    lcd_write_data(0x00);
    
    lcd_write_command(0xFF);
    lcd_write_data(0x60);
    lcd_write_data(0x01);
    lcd_write_data(0x04);
    
    lcd_write_command(0xC3);
    lcd_write_data(0x13);
    lcd_write_command(0xC4);
    lcd_write_data(0x13);
    
    lcd_write_command(0xC9);
    lcd_write_data(0x22);
    
    lcd_write_command(0xBE);
    lcd_write_data(0x11);
    
    lcd_write_command(0xE1);
    lcd_write_data(0x10);
    lcd_write_data(0x0E);
    
    lcd_write_command(0xDF);
    lcd_write_data(0x21);
    lcd_write_data(0x0C);
    lcd_write_data(0x02);
    
    lcd_write_command(0xF0);
    lcd_write_data(0x45);
    lcd_write_data(0x09);
    lcd_write_data(0x08);
    lcd_write_data(0x08);
    lcd_write_data(0x26);
    lcd_write_data(0x2A);
    
    lcd_write_command(0xF1);
    lcd_write_data(0x43);
    lcd_write_data(0x70);
    lcd_write_data(0x72);
    lcd_write_data(0x36);
    lcd_write_data(0x37);
    lcd_write_data(0x6F);
    
    lcd_write_command(0xF2);
    lcd_write_data(0x45);
    lcd_write_data(0x09);
    lcd_write_data(0x08);
    lcd_write_data(0x08);
    lcd_write_data(0x26);
    lcd_write_data(0x2A);
    
    lcd_write_command(0xF3);
    lcd_write_data(0x43);
    lcd_write_data(0x70);
    lcd_write_data(0x72);
    lcd_write_data(0x36);
    lcd_write_data(0x37);
    lcd_write_data(0x6F);
    
    lcd_write_command(0xED);
    lcd_write_data(0x1B);
    lcd_write_data(0x0B);
    
    lcd_write_command(0xAE);
    lcd_write_data(0x77);
    
    lcd_write_command(0xCD);
    lcd_write_data(0x63);
    
    lcd_write_command(0x70);
    lcd_write_data(0x07);
    lcd_write_data(0x07);
    lcd_write_data(0x04);
    lcd_write_data(0x0E);
    lcd_write_data(0x0F);
    lcd_write_data(0x09);
    lcd_write_data(0x07);
    lcd_write_data(0x08);
    lcd_write_data(0x03);
    
    lcd_write_command(0xE8);
    lcd_write_data(0x34);
    
    lcd_write_command(0x62);
    lcd_write_data(0x18);
    lcd_write_data(0x0D);
    lcd_write_data(0x71);
    lcd_write_data(0xED);
    lcd_write_data(0x70);
    lcd_write_data(0x70);
    lcd_write_data(0x18);
    lcd_write_data(0x0F);
    lcd_write_data(0x71);
    lcd_write_data(0xEF);
    lcd_write_data(0x70);
    lcd_write_data(0x70);
    
    lcd_write_command(0x63);
    lcd_write_data(0x18);
    lcd_write_data(0x11);
    lcd_write_data(0x71);
    lcd_write_data(0xF1);
    lcd_write_data(0x70);
    lcd_write_data(0x70);
    lcd_write_data(0x18);
    lcd_write_data(0x13);
    lcd_write_data(0x71);
    lcd_write_data(0xF3);
    lcd_write_data(0x70);
    lcd_write_data(0x70);
    
    lcd_write_command(0x64);
    lcd_write_data(0x28);
    lcd_write_data(0x29);
    lcd_write_data(0xF1);
    lcd_write_data(0x01);
    lcd_write_data(0xF1);
    lcd_write_data(0x00);
    lcd_write_data(0x07);
    
    lcd_write_command(0x66);
    lcd_write_data(0x3C);
    lcd_write_data(0x00);
    lcd_write_data(0xCD);
    lcd_write_data(0x67);
    lcd_write_data(0x45);
    lcd_write_data(0x45);
    lcd_write_data(0x10);
    lcd_write_data(0x00);
    lcd_write_data(0x00);
    lcd_write_data(0x00);
    
    lcd_write_command(0x67);
    lcd_write_data(0x00);
    lcd_write_data(0x3C);
    lcd_write_data(0x00);
    lcd_write_data(0x00);
    lcd_write_data(0x00);
    lcd_write_data(0x01);
    lcd_write_data(0x54);
    lcd_write_data(0x10);
    lcd_write_data(0x32);
    lcd_write_data(0x98);
    
    lcd_write_command(0x74);
    lcd_write_data(0x10);
    lcd_write_data(0x85);
    lcd_write_data(0x80);
    lcd_write_data(0x00);
    lcd_write_data(0x00);
    lcd_write_data(0x4E);
    lcd_write_data(0x00);
    
    lcd_write_command(0x98);
    lcd_write_data(0x3E);
    lcd_write_data(0x07);
    
    lcd_write_command(0x35);
    lcd_write_command(0x21);
    
    lcd_write_command(0x11);
    sleep_ms(120);
    
    lcd_write_command(0x29);
    sleep_ms(20);
}

static void lcd_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    lcd_write_command(0x2A);
    lcd_write_data(x0 >> 8);
    lcd_write_data(x0 & 0xFF);
    lcd_write_data(x1 >> 8);
    lcd_write_data(x1 & 0xFF);
    
    lcd_write_command(0x2B);
    lcd_write_data(y0 >> 8);
    lcd_write_data(y0 & 0xFF);
    lcd_write_data(y1 >> 8);
    lcd_write_data(y1 & 0xFF);
    
    lcd_write_command(0x2C);
}

static void lcd_clear(uint16_t color) {
    lcd_set_window(0, 0, 239, 239);
    gpio_put(LCD_DC_PIN, 1);
    
    for (int i = 0; i < 240 * 240; i++) {
        lcd_write_data16(color);
    }
}

static void lcd_draw_pixel(uint16_t x, uint16_t y, uint16_t color) {
    if (x >= LCD_WIDTH || y >= LCD_HEIGHT) return;
    
    lcd_set_window(x, y, x, y);
    lcd_write_data16(color);
}

static void lcd_draw_char(uint16_t x, uint16_t y, char c, uint16_t color, uint16_t bg, uint8_t scale) {
    if (c < ' ' || c > 'Z') return;
    
    const uint8_t *char_data = font5x7[c - ' '];
    
    for (int i = 0; i < 5; i++) {
        uint8_t line = char_data[i];
        for (int j = 0; j < 8; j++) {
            if (line & (1 << j)) {
                for (int sx = 0; sx < scale; sx++) {
                    for (int sy = 0; sy < scale; sy++) {
                        lcd_draw_pixel(x + i * scale + sx, y + j * scale + sy, color);
                    }
                }
            } else if (bg != 0xFFFF) {  // 0xFFFF = transparent background
                for (int sx = 0; sx < scale; sx++) {
                    for (int sy = 0; sy < scale; sy++) {
                        lcd_draw_pixel(x + i * scale + sx, y + j * scale + sy, bg);
                    }
                }
            }
        }
    }
}

static void lcd_draw_string(uint16_t x, uint16_t y, const char *str, uint16_t color, uint16_t bg, uint8_t scale) {
    while (*str) {
        lcd_draw_char(x, y, *str, color, bg, scale);
        x += 6 * scale;
        str++;
    }
}

// I2C functions for IMU
static void i2c_write_register(uint8_t addr, uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    i2c_write_blocking(i2c1, addr, buf, 2, false);
}

static uint8_t i2c_read_register(uint8_t addr, uint8_t reg) {
    uint8_t value;
    i2c_write_blocking(i2c1, addr, &reg, 1, true);
    i2c_read_blocking(i2c1, addr, &value, 1, false);
    return value;
}

static int16_t i2c_read_16bit(uint8_t addr, uint8_t reg_l) {
    uint8_t low = i2c_read_register(addr, reg_l);
    uint8_t high = i2c_read_register(addr, reg_l + 1);
    return (int16_t)((high << 8) | low);
}

// IMU functions
static bool qmi8658_init(void) {
    // Check WHO_AM_I register
    uint8_t who_am_i = i2c_read_register(QMI8658_ADDR, QMI8658_WHO_AM_I);
    printf("QMI8658 WHO_AM_I: 0x%02X\n", who_am_i);
    
    if (who_am_i != 0x05) {
        printf("QMI8658 not found!\n");
        return false;
    }
    
    // Reset
    i2c_write_register(QMI8658_ADDR, QMI8658_CTRL1, 0x60);
    sleep_ms(10);
    
    // Configure accelerometer: 8g range, 1000Hz ODR
    i2c_write_register(QMI8658_ADDR, QMI8658_CTRL2, 0x0F);
    
    // Configure gyroscope: 2048 dps range, 1000Hz ODR
    i2c_write_register(QMI8658_ADDR, QMI8658_CTRL3, 0x6F);
    
    // Enable sensors
    i2c_write_register(QMI8658_ADDR, QMI8658_CTRL7, 0x03);
    
    return true;
}

static void qmi8658_read_data(imu_data_t *data) {
    // Read accelerometer data
    int16_t ax = i2c_read_16bit(QMI8658_ADDR, QMI8658_AX_L);
    int16_t ay = i2c_read_16bit(QMI8658_ADDR, QMI8658_AY_L);
    int16_t az = i2c_read_16bit(QMI8658_ADDR, QMI8658_AZ_L);
    
    // Read gyroscope data
    int16_t gx = i2c_read_16bit(QMI8658_ADDR, QMI8658_GX_L);
    int16_t gy = i2c_read_16bit(QMI8658_ADDR, QMI8658_GY_L);
    int16_t gz = i2c_read_16bit(QMI8658_ADDR, QMI8658_GZ_L);
    
    // Read temperature
    int16_t temp = i2c_read_16bit(QMI8658_ADDR, QMI8658_TEMP_L);
    
    // Convert to physical units
    // Accelerometer: 8g range, 16-bit signed
    data->accel_x = (ax / 4096.0f);  // g
    data->accel_y = (ay / 4096.0f);  // g
    data->accel_z = (az / 4096.0f);  // g
    
    // Gyroscope: 2048 dps range, 16-bit signed
    data->gyro_x = (gx / 16.0f);  // dps
    data->gyro_y = (gy / 16.0f);  // dps
    data->gyro_z = (gz / 16.0f);  // dps
    
    // Temperature
    data->temperature = (temp / 256.0f) + 25.0f;  // Celsius
}

// Battery voltage reading
static float read_battery_voltage(void) {
    uint16_t adc_value = adc_read();
    // Convert to voltage (3.3V reference, 12-bit ADC)
    // Battery voltage is divided by 2 before ADC
    float voltage = (adc_value * 3.3f / 4095.0f) * 2.0f;
    return voltage;
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    
    printf("\n=== RP2350-LCD-1.28 IMU Display Demo ===\n");
    printf("Displaying 6-axis IMU data and battery voltage\n\n");
    
    // Initialize I2C for IMU
    i2c_init(i2c1, 400000);  // 400kHz
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    
    // Initialize ADC for battery voltage
    adc_init();
    adc_gpio_init(BAT_ADC_PIN);
    adc_select_input(BAT_ADC_CHANNEL);
    
    // Initialize LCD control pins
    gpio_init(LCD_DC_PIN);
    gpio_set_dir(LCD_DC_PIN, GPIO_OUT);
    
    gpio_init(LCD_CS_PIN);
    gpio_set_dir(LCD_CS_PIN, GPIO_OUT);
    gpio_put(LCD_CS_PIN, 1);
    
    gpio_init(LCD_RST_PIN);
    gpio_set_dir(LCD_RST_PIN, GPIO_OUT);
    
    // Initialize SPI
    spi_init(spi1, 62500000);
    gpio_set_function(LCD_CLK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(LCD_MOSI_PIN, GPIO_FUNC_SPI);
    
    // Initialize backlight with PWM
    gpio_set_function(LCD_BL_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(LCD_BL_PIN);
    pwm_set_wrap(slice, 255);
    pwm_set_gpio_level(LCD_BL_PIN, 200);  // ~80% brightness
    pwm_set_enabled(slice, true);
    
    // Initialize LCD
    printf("Initializing LCD...\n");
    lcd_reset();
    lcd_init_reg();
    lcd_clear(0x0000);  // Clear to black
    
    // Initialize IMU
    printf("Initializing IMU...\n");
    if (!qmi8658_init()) {
        printf("Failed to initialize IMU!\n");
        // Continue anyway to show battery voltage
    }
    
    printf("Starting sensor display...\n");
    
    // Display title
    lcd_draw_string(50, 10, "RP2350 IMU DEMO", 0xFFFF, 0x0000, 2);
    
    // Labels
    lcd_draw_string(10, 50, "ACCEL:", 0x07E0, 0x0000, 1);
    lcd_draw_string(10, 90, "GYRO:", 0x001F, 0x0000, 1);
    lcd_draw_string(10, 130, "TEMP:", 0xFFE0, 0x0000, 1);
    lcd_draw_string(10, 150, "BATT:", 0xF800, 0x0000, 1);
    
    imu_data_t imu_data;
    char buffer[64];
    
    while (1) {
        // Read IMU data
        qmi8658_read_data(&imu_data);
        
        // Read battery voltage
        float battery_v = read_battery_voltage();
        
        // Display accelerometer data
        snprintf(buffer, sizeof(buffer), "X:%+.2F G  ", imu_data.accel_x);
        lcd_draw_string(10, 60, buffer, 0xFFFF, 0x0000, 1);
        
        snprintf(buffer, sizeof(buffer), "Y:%+.2F G  ", imu_data.accel_y);
        lcd_draw_string(10, 70, buffer, 0xFFFF, 0x0000, 1);
        
        snprintf(buffer, sizeof(buffer), "Z:%+.2F G  ", imu_data.accel_z);
        lcd_draw_string(10, 80, buffer, 0xFFFF, 0x0000, 1);
        
        // Display gyroscope data
        snprintf(buffer, sizeof(buffer), "X:%+.1F DPS  ", imu_data.gyro_x);
        lcd_draw_string(10, 100, buffer, 0xFFFF, 0x0000, 1);
        
        snprintf(buffer, sizeof(buffer), "Y:%+.1F DPS  ", imu_data.gyro_y);
        lcd_draw_string(10, 110, buffer, 0xFFFF, 0x0000, 1);
        
        snprintf(buffer, sizeof(buffer), "Z:%+.1F DPS  ", imu_data.gyro_z);
        lcd_draw_string(10, 120, buffer, 0xFFFF, 0x0000, 1);
        
        // Display temperature
        snprintf(buffer, sizeof(buffer), "%.1F C  ", imu_data.temperature);
        lcd_draw_string(60, 130, buffer, 0xFFFF, 0x0000, 1);
        
        // Display battery voltage
        snprintf(buffer, sizeof(buffer), "%.2F V  ", battery_v);
        lcd_draw_string(60, 150, buffer, 0xFFFF, 0x0000, 1);
        
        // Print to console
        printf("\rAcc: X=%+.2fg Y=%+.2fg Z=%+.2fg | Gyro: X=%+.1f Y=%+.1f Z=%+.1f | Temp=%.1fC | Batt=%.2fV",
               imu_data.accel_x, imu_data.accel_y, imu_data.accel_z,
               imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z,
               imu_data.temperature, battery_v);
        
        sleep_ms(100);  // Update at 10Hz
    }
    
    return 0;
}