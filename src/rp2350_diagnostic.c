/**
 * RP2350-LCD-1.28 Diagnostic Test
 * Minimal test to verify LCD and basic functionality
 */

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "hardware/pwm.h"
#include <stdio.h>

// Pin definitions for RP2350-LCD-1.28
#define LCD_CS_PIN      9
#define LCD_DC_PIN      8  
#define LCD_RST_PIN     12
#define LCD_BL_PIN      25
#define LCD_SCK_PIN     10
#define LCD_MOSI_PIN    11

#define BUTTON_A_PIN    15
#define BUTTON_B_PIN    17

// LED pin (usually pin 25 or board LED)
#define LED_PIN         25

// GC9A01 Commands
#define GC9A01_SWRESET  0x01
#define GC9A01_SLPOUT   0x11
#define GC9A01_DISPON   0x29
#define GC9A01_CASET    0x2A
#define GC9A01_RASET    0x2B
#define GC9A01_RAMWR    0x2C
#define GC9A01_MADCTL   0x36
#define GC9A01_COLMOD   0x3A

// Helper functions
static inline void lcd_cs_select(void) {
    gpio_put(LCD_CS_PIN, 0);
}

static inline void lcd_cs_deselect(void) {
    gpio_put(LCD_CS_PIN, 1);
}

static inline void lcd_dc_command(void) {
    gpio_put(LCD_DC_PIN, 0);
}

static inline void lcd_dc_data(void) {
    gpio_put(LCD_DC_PIN, 1);
}

static void lcd_write_command(uint8_t cmd) {
    lcd_cs_select();
    lcd_dc_command();
    spi_write_blocking(spi1, &cmd, 1);
    lcd_cs_deselect();
}

static void lcd_write_data(uint8_t data) {
    lcd_cs_select();
    lcd_dc_data();
    spi_write_blocking(spi1, &data, 1);
    lcd_cs_deselect();
}

static void lcd_init_minimal(void) {
    // Initialize SPI
    spi_init(spi1, 10 * 1000 * 1000); // Start with 10MHz for safety
    gpio_set_function(LCD_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(LCD_MOSI_PIN, GPIO_FUNC_SPI);
    
    // Initialize control pins
    gpio_init(LCD_CS_PIN);
    gpio_set_dir(LCD_CS_PIN, GPIO_OUT);
    gpio_put(LCD_CS_PIN, 1);
    
    gpio_init(LCD_DC_PIN);
    gpio_set_dir(LCD_DC_PIN, GPIO_OUT);
    
    gpio_init(LCD_RST_PIN);
    gpio_set_dir(LCD_RST_PIN, GPIO_OUT);
    
    // Initialize backlight with maximum brightness for testing
    gpio_init(LCD_BL_PIN);
    gpio_set_dir(LCD_BL_PIN, GPIO_OUT);
    gpio_put(LCD_BL_PIN, 1);  // Simple GPIO high first
    
    printf("Step 1: GPIO initialized\n");
    
    // Hardware reset
    gpio_put(LCD_RST_PIN, 0);
    sleep_ms(100);
    gpio_put(LCD_RST_PIN, 1);
    sleep_ms(100);
    
    printf("Step 2: Hardware reset complete\n");
    
    // Software reset
    lcd_write_command(GC9A01_SWRESET);
    sleep_ms(150);
    
    printf("Step 3: Software reset complete\n");
    
    // Sleep out
    lcd_write_command(GC9A01_SLPOUT);
    sleep_ms(120);
    
    printf("Step 4: Sleep out complete\n");
    
    // Color mode - 16-bit color
    lcd_write_command(GC9A01_COLMOD);
    lcd_write_data(0x55);
    
    // Memory access control
    lcd_write_command(GC9A01_MADCTL);
    lcd_write_data(0x00);
    
    // Display on
    lcd_write_command(GC9A01_DISPON);
    sleep_ms(20);
    
    printf("Step 5: Display enabled\n");
    
    // Now set up PWM for backlight
    gpio_set_function(LCD_BL_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(LCD_BL_PIN);
    pwm_set_wrap(slice_num, 255);
    pwm_set_gpio_level(LCD_BL_PIN, 255); // Maximum brightness
    pwm_set_enabled(slice_num, true);
    
    printf("Step 6: Backlight PWM enabled at max brightness\n");
}

static void lcd_fill_screen(uint16_t color) {
    // Set address window to full screen
    lcd_write_command(GC9A01_CASET);
    lcd_write_data(0x00);
    lcd_write_data(0x00);
    lcd_write_data(0x00);
    lcd_write_data(0xEF); // 239
    
    lcd_write_command(GC9A01_RASET);
    lcd_write_data(0x00);
    lcd_write_data(0x00);
    lcd_write_data(0x00);
    lcd_write_data(0xEF); // 239
    
    lcd_write_command(GC9A01_RAMWR);
    
    uint8_t color_data[2];
    color_data[0] = color >> 8;
    color_data[1] = color & 0xFF;
    
    lcd_cs_select();
    lcd_dc_data();
    
    // Fill entire screen
    for (int i = 0; i < 240 * 240; i++) {
        spi_write_blocking(spi1, color_data, 2);
    }
    
    lcd_cs_deselect();
}

static void test_pattern(void) {
    // Draw different colored rectangles in quadrants
    lcd_write_command(GC9A01_CASET);
    lcd_write_data(0x00);
    lcd_write_data(0x00);
    lcd_write_data(0x00);
    lcd_write_data(0x77); // 119
    
    lcd_write_command(GC9A01_RASET);
    lcd_write_data(0x00);
    lcd_write_data(0x00);
    lcd_write_data(0x00);
    lcd_write_data(0x77); // 119
    
    lcd_write_command(GC9A01_RAMWR);
    
    lcd_cs_select();
    lcd_dc_data();
    
    // Red quadrant
    uint8_t red[2] = {0xF8, 0x00};
    for (int i = 0; i < 120 * 120; i++) {
        spi_write_blocking(spi1, red, 2);
    }
    
    lcd_cs_deselect();
}

int main(void) {
    // Initialize stdio for USB/UART output
    stdio_init_all();
    
    // Wait a bit for USB to enumerate
    sleep_ms(2000);
    
    printf("\n=== RP2350-LCD-1.28 Diagnostic Test ===\n");
    printf("This will test the LCD display step by step\n");
    printf("=======================================\n\n");
    
    // Initialize buttons with pull-ups
    gpio_init(BUTTON_A_PIN);
    gpio_set_dir(BUTTON_A_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_A_PIN);
    
    gpio_init(BUTTON_B_PIN);
    gpio_set_dir(BUTTON_B_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_B_PIN);
    
    printf("Buttons initialized\n");
    
    // Initialize onboard LED for heartbeat
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    
    printf("Starting LCD initialization...\n");
    
    // Initialize LCD
    lcd_init_minimal();
    
    printf("LCD initialization complete!\n");
    printf("Attempting to fill screen with colors...\n");
    
    // Test sequence
    int color_index = 0;
    uint16_t colors[] = {
        0xF800,  // Red
        0x07E0,  // Green  
        0x001F,  // Blue
        0xFFFF,  // White
        0xFFE0,  // Yellow
        0xF81F,  // Magenta
        0x07FF,  // Cyan
        0xFD20   // Orange
    };
    const char *color_names[] = {
        "RED", "GREEN", "BLUE", "WHITE", 
        "YELLOW", "MAGENTA", "CYAN", "ORANGE"
    };
    
    while (1) {
        // LED heartbeat
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(100);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(100);
        
        // Fill screen with current color
        printf("Filling screen with %s (0x%04X)\n", 
               color_names[color_index], colors[color_index]);
        lcd_fill_screen(colors[color_index]);
        
        // Check button A to cycle colors
        if (gpio_get(BUTTON_A_PIN) == 0) {
            printf("Button A pressed - next color\n");
            color_index = (color_index + 1) % 8;
            sleep_ms(200); // Debounce
        }
        
        // Check button B to run test pattern
        if (gpio_get(BUTTON_B_PIN) == 0) {
            printf("Button B pressed - test pattern\n");
            test_pattern();
            sleep_ms(200); // Debounce
        }
        
        // Also cycle automatically every 2 seconds
        static int counter = 0;
        if (++counter >= 10) {
            counter = 0;
            color_index = (color_index + 1) % 8;
        }
        
        sleep_ms(200);
    }
    
    return 0;
}