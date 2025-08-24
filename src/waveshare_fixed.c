/**
 * RP2350-LCD-1.28 Fixed Demo
 * Based on official Waveshare source code
 */

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "hardware/pwm.h"
#include <stdio.h>
#include <string.h>

// LCD Pin definitions (from Waveshare official code)
#define LCD_DC_PIN      8
#define LCD_CS_PIN      9
#define LCD_CLK_PIN     10
#define LCD_MOSI_PIN    11
#define LCD_RST_PIN     12
#define LCD_BL_PIN      25

// CRITICAL: CS is held LOW after reset and never toggled!

static void lcd_write_command(uint8_t cmd) {
    gpio_put(LCD_DC_PIN, 0);  // Command mode
    // CS is already low - don't toggle it!
    spi_write_blocking(spi1, &cmd, 1);
}

static void lcd_write_data(uint8_t data) {
    gpio_put(LCD_DC_PIN, 1);  // Data mode
    // CS is already low - don't toggle it!
    spi_write_blocking(spi1, &data, 1);
}

static void lcd_reset(void) {
    gpio_put(LCD_RST_PIN, 1);
    sleep_ms(100);
    gpio_put(LCD_RST_PIN, 0);
    sleep_ms(100);
    gpio_put(LCD_RST_PIN, 1);
    
    // CRITICAL: Set CS LOW and keep it LOW!
    gpio_put(LCD_CS_PIN, 0);
    
    sleep_ms(100);
}

static void lcd_init_reg(void) {
    // Exact initialization sequence from Waveshare
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
    
    // Memory Access Control - 0x08 for vertical screen
    lcd_write_command(0x36);
    lcd_write_data(0x08);
    
    // Pixel Format
    lcd_write_command(0x3A);
    lcd_write_data(0x05);  // 16-bit color
    
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
    
    // Tearing effect line ON
    lcd_write_command(0x35);
    // Display inversion ON
    lcd_write_command(0x21);
    
    // Sleep out
    lcd_write_command(0x11);
    sleep_ms(120);
    
    // Display ON
    lcd_write_command(0x29);
    sleep_ms(20);
}

static void lcd_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    // Column address
    lcd_write_command(0x2A);
    lcd_write_data(x0 >> 8);
    lcd_write_data(x0 & 0xFF);
    lcd_write_data(x1 >> 8);
    lcd_write_data(x1 & 0xFF);
    
    // Row address
    lcd_write_command(0x2B);
    lcd_write_data(y0 >> 8);
    lcd_write_data(y0 & 0xFF);
    lcd_write_data(y1 >> 8);
    lcd_write_data(y1 & 0xFF);
    
    // Write memory
    lcd_write_command(0x2C);
}

static void lcd_clear(uint16_t color) {
    lcd_set_window(0, 0, 239, 239);
    
    gpio_put(LCD_DC_PIN, 1);  // Data mode
    
    uint8_t data[2];
    data[0] = color >> 8;
    data[1] = color & 0xFF;
    
    for (int i = 0; i < 240 * 240; i++) {
        spi_write_blocking(spi1, data, 2);
    }
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    
    printf("\n=== Waveshare RP2350-LCD-1.28 Fixed Demo ===\n");
    printf("Using correct CS handling from official code\n\n");
    
    // Initialize control pins
    gpio_init(LCD_DC_PIN);
    gpio_set_dir(LCD_DC_PIN, GPIO_OUT);
    
    gpio_init(LCD_CS_PIN);
    gpio_set_dir(LCD_CS_PIN, GPIO_OUT);
    gpio_put(LCD_CS_PIN, 1);  // Start with CS high
    
    gpio_init(LCD_RST_PIN);
    gpio_set_dir(LCD_RST_PIN, GPIO_OUT);
    
    // Initialize SPI
    spi_init(spi1, 62500000);  // 62.5MHz as per Waveshare
    gpio_set_function(LCD_CLK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(LCD_MOSI_PIN, GPIO_FUNC_SPI);
    
    // Initialize backlight
    gpio_init(LCD_BL_PIN);
    gpio_set_dir(LCD_BL_PIN, GPIO_OUT);
    gpio_put(LCD_BL_PIN, 1);
    
    // Set up PWM for backlight
    gpio_set_function(LCD_BL_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(LCD_BL_PIN);
    pwm_set_wrap(slice, 255);
    pwm_set_gpio_level(LCD_BL_PIN, 255);  // Full brightness
    pwm_set_enabled(slice, true);
    
    printf("Initializing LCD...\n");
    
    // Reset and initialize
    lcd_reset();  // This sets CS LOW and keeps it LOW!
    lcd_init_reg();
    
    printf("LCD initialized, cycling colors...\n");
    
    // Color cycle
    uint16_t colors[] = {
        0xF800,  // Red
        0x07E0,  // Green
        0x001F,  // Blue
        0xFFFF,  // White
        0x0000,  // Black
        0xFFE0,  // Yellow
        0xF81F,  // Magenta
        0x07FF   // Cyan
    };
    
    const char *names[] = {
        "RED", "GREEN", "BLUE", "WHITE",
        "BLACK", "YELLOW", "MAGENTA", "CYAN"
    };
    
    int idx = 0;
    while (1) {
        printf("Displaying: %s\n", names[idx]);
        lcd_clear(colors[idx]);
        sleep_ms(2000);
        idx = (idx + 1) % 8;
    }
    
    return 0;
}