/**
 * RP2350-LCD-1.28 SPI Communication Test
 * Debug version to identify the exact issue
 */

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "hardware/pwm.h"
#include <stdio.h>
#include <string.h>

// LCD pins - let's verify these are correct
#define LCD_DC_PIN      8
#define LCD_CS_PIN      9
#define LCD_SCLK_PIN    10
#define LCD_MOSI_PIN    11
#define LCD_RST_PIN     12
#define LCD_BL_PIN      25

// Test with both SPI ports to be sure
static spi_inst_t *current_spi = spi1;

static void gpio_test(void) {
    printf("\n=== GPIO Pin Test ===\n");
    
    // Test each pin individually
    printf("Testing LCD_DC_PIN (GPIO%d)...\n", LCD_DC_PIN);
    gpio_init(LCD_DC_PIN);
    gpio_set_dir(LCD_DC_PIN, GPIO_OUT);
    for(int i = 0; i < 3; i++) {
        gpio_put(LCD_DC_PIN, 1);
        sleep_ms(100);
        gpio_put(LCD_DC_PIN, 0);
        sleep_ms(100);
    }
    
    printf("Testing LCD_CS_PIN (GPIO%d)...\n", LCD_CS_PIN);
    gpio_init(LCD_CS_PIN);
    gpio_set_dir(LCD_CS_PIN, GPIO_OUT);
    gpio_put(LCD_CS_PIN, 1);  // CS is active low
    
    printf("Testing LCD_RST_PIN (GPIO%d)...\n", LCD_RST_PIN);
    gpio_init(LCD_RST_PIN);
    gpio_set_dir(LCD_RST_PIN, GPIO_OUT);
    gpio_put(LCD_RST_PIN, 1);
    
    printf("Testing LCD_BL_PIN (GPIO%d)...\n", LCD_BL_PIN);
    gpio_init(LCD_BL_PIN);
    gpio_set_dir(LCD_BL_PIN, GPIO_OUT);
    gpio_put(LCD_BL_PIN, 1);
    
    printf("GPIO test complete\n");
}

static void spi_init_test(void) {
    printf("\n=== SPI Initialization Test ===\n");
    
    // Try different SPI configurations
    uint32_t speeds[] = {1000000, 10000000, 20000000, 40000000};  // 1MHz to 40MHz
    
    for(int i = 0; i < 4; i++) {
        printf("Testing SPI at %lu Hz...\n", (unsigned long)speeds[i]);
        
        spi_deinit(current_spi);
        spi_init(current_spi, speeds[i]);
        
        // Set SPI format (Mode 0: CPOL=0, CPHA=0)
        spi_set_format(current_spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
        
        gpio_set_function(LCD_SCLK_PIN, GPIO_FUNC_SPI);
        gpio_set_function(LCD_MOSI_PIN, GPIO_FUNC_SPI);
        
        // Try to send some data
        uint8_t test_data[] = {0xAA, 0x55, 0xFF, 0x00};
        gpio_put(LCD_CS_PIN, 0);
        int result = spi_write_blocking(current_spi, test_data, 4);
        gpio_put(LCD_CS_PIN, 1);
        
        printf("  Sent %d bytes\n", result);
        sleep_ms(10);
    }
}

static void lcd_write_byte(uint8_t dc, uint8_t data) {
    gpio_put(LCD_CS_PIN, 0);
    gpio_put(LCD_DC_PIN, dc);
    spi_write_blocking(current_spi, &data, 1);
    gpio_put(LCD_CS_PIN, 1);
}

static void lcd_reset_test(void) {
    printf("\n=== LCD Reset Sequence ===\n");
    
    printf("Performing hardware reset...\n");
    gpio_put(LCD_RST_PIN, 1);
    sleep_ms(10);
    gpio_put(LCD_RST_PIN, 0);
    sleep_ms(20);
    gpio_put(LCD_RST_PIN, 1);
    sleep_ms(120);
    printf("Hardware reset complete\n");
}

static void lcd_basic_init(void) {
    printf("\n=== Basic LCD Initialization ===\n");
    
    // Software reset
    printf("Sending software reset (0x01)...\n");
    lcd_write_byte(0, 0x01);
    sleep_ms(150);
    
    // Sleep out
    printf("Sending sleep out (0x11)...\n");
    lcd_write_byte(0, 0x11);
    sleep_ms(120);
    
    // Display on
    printf("Sending display on (0x29)...\n");
    lcd_write_byte(0, 0x29);
    sleep_ms(20);
    
    printf("Basic init complete\n");
}

static void lcd_fill_test(void) {
    printf("\n=== LCD Fill Test ===\n");
    
    // Set column address (0x2A)
    lcd_write_byte(0, 0x2A);
    lcd_write_byte(1, 0x00);
    lcd_write_byte(1, 0x00);
    lcd_write_byte(1, 0x00);
    lcd_write_byte(1, 0xEF);
    
    // Set row address (0x2B)
    lcd_write_byte(0, 0x2B);
    lcd_write_byte(1, 0x00);
    lcd_write_byte(1, 0x00);
    lcd_write_byte(1, 0x00);
    lcd_write_byte(1, 0xEF);
    
    // Memory write (0x2C)
    lcd_write_byte(0, 0x2C);
    
    printf("Filling screen with red (RGB565: 0xF800)...\n");
    
    gpio_put(LCD_CS_PIN, 0);
    gpio_put(LCD_DC_PIN, 1);
    
    // Fill with red pixels
    uint8_t red_pixel[2] = {0xF8, 0x00};
    for(int i = 0; i < 240 * 240; i++) {
        spi_write_blocking(current_spi, red_pixel, 2);
        
        // Progress indicator every 10%
        if(i % 5760 == 0) {
            printf("  %d%%\n", (i * 100) / (240 * 240));
        }
    }
    
    gpio_put(LCD_CS_PIN, 1);
    printf("Fill complete\n");
}

static void backlight_test(void) {
    printf("\n=== Backlight PWM Test ===\n");
    
    // First try simple GPIO
    printf("Testing GPIO control...\n");
    gpio_init(LCD_BL_PIN);
    gpio_set_dir(LCD_BL_PIN, GPIO_OUT);
    
    for(int i = 0; i < 3; i++) {
        printf("  Backlight ON\n");
        gpio_put(LCD_BL_PIN, 1);
        sleep_ms(500);
        printf("  Backlight OFF\n");
        gpio_put(LCD_BL_PIN, 0);
        sleep_ms(500);
    }
    
    // Now setup PWM
    printf("Setting up PWM control...\n");
    gpio_set_function(LCD_BL_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(LCD_BL_PIN);
    pwm_set_wrap(slice, 255);
    
    // Fade in/out
    printf("Fading backlight...\n");
    pwm_set_enabled(slice, true);
    
    for(int level = 0; level <= 255; level += 5) {
        pwm_set_gpio_level(LCD_BL_PIN, level);
        sleep_ms(20);
    }
    
    for(int level = 255; level >= 0; level -= 5) {
        pwm_set_gpio_level(LCD_BL_PIN, level);
        sleep_ms(20);
    }
    
    // Leave at full brightness
    pwm_set_gpio_level(LCD_BL_PIN, 255);
    printf("Backlight test complete\n");
}

int main(void) {
    stdio_init_all();
    
    // Longer wait for USB serial
    for(int i = 0; i < 30; i++) {
        sleep_ms(100);
        printf(".");
    }
    
    printf("\n\n");
    printf("=====================================\n");
    printf("  RP2350-LCD-1.28 SPI Debug Test    \n");
    printf("=====================================\n");
    printf("\n");
    printf("This test will verify each component\n");
    printf("Watch the serial output and LCD\n");
    printf("\n");
    
    // Test 1: GPIO pins
    gpio_test();
    sleep_ms(1000);
    
    // Test 2: Backlight
    backlight_test();
    sleep_ms(1000);
    
    // Test 3: SPI initialization
    spi_init_test();
    sleep_ms(1000);
    
    // Test 4: LCD reset
    lcd_reset_test();
    sleep_ms(1000);
    
    // Test 5: Basic LCD init
    lcd_basic_init();
    sleep_ms(1000);
    
    // Test 6: Try to fill screen
    lcd_fill_test();
    
    printf("\n=== Test Complete ===\n");
    printf("If you see a red screen, SPI works!\n");
    printf("If backlight changes but no red, SPI issue\n");
    printf("Check serial output for any errors\n");
    
    // Keep running and flash LED
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    
    while(1) {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(500);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(500);
    }
    
    return 0;
}