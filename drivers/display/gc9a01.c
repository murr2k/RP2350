/**
 * GC9A01 Round LCD Display Driver Implementation
 * For Waveshare RP2350-LCD-1.28 (240x240 pixels)
 */

#include "gc9a01.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

// GC9A01 Commands
#define GC9A01_SWRESET      0x01
#define GC9A01_RDDID        0x04
#define GC9A01_RDDST        0x09
#define GC9A01_SLPIN        0x10
#define GC9A01_SLPOUT       0x11
#define GC9A01_PTLON        0x12
#define GC9A01_NORON        0x13
#define GC9A01_INVOFF       0x20
#define GC9A01_INVON        0x21
#define GC9A01_DISPOFF      0x28
#define GC9A01_DISPON       0x29
#define GC9A01_CASET        0x2A
#define GC9A01_RASET        0x2B
#define GC9A01_RAMWR        0x2C
#define GC9A01_RAMRD        0x2E
#define GC9A01_PTLAR        0x30
#define GC9A01_COLMOD       0x3A
#define GC9A01_MADCTL       0x36

// Memory Access Control
#define MADCTL_MY           0x80
#define MADCTL_MX           0x40
#define MADCTL_MV           0x20
#define MADCTL_ML           0x10
#define MADCTL_RGB          0x00
#define MADCTL_BGR          0x08
#define MADCTL_MH           0x04

// Static instance
static gc9a01_t g_lcd;

// Helper functions
static inline void gc9a01_cs_select(void) {
    gpio_put(g_lcd.cs_pin, 0);
}

static inline void gc9a01_cs_deselect(void) {
    gpio_put(g_lcd.cs_pin, 1);
}

static inline void gc9a01_dc_command(void) {
    gpio_put(g_lcd.dc_pin, 0);
}

static inline void gc9a01_dc_data(void) {
    gpio_put(g_lcd.dc_pin, 1);
}

static inline void gc9a01_reset_active(void) {
    gpio_put(g_lcd.rst_pin, 0);
}

static inline void gc9a01_reset_inactive(void) {
    gpio_put(g_lcd.rst_pin, 1);
}

static void gc9a01_write_command(uint8_t cmd) {
    gc9a01_cs_select();
    gc9a01_dc_command();
    spi_write_blocking(g_lcd.spi_inst, &cmd, 1);
    gc9a01_cs_deselect();
}

static void gc9a01_write_data(uint8_t data) {
    gc9a01_cs_select();
    gc9a01_dc_data();
    spi_write_blocking(g_lcd.spi_inst, &data, 1);
    gc9a01_cs_deselect();
}

// Commented out for now - will be used for future optimizations
// static void gc9a01_write_data_bulk(const uint8_t *data, size_t len) {
//     gc9a01_cs_select();
//     gc9a01_dc_data();
//     spi_write_blocking(g_lcd.spi_inst, data, len);
//     gc9a01_cs_deselect();
// }

static void gc9a01_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    // Column address set
    gc9a01_write_command(GC9A01_CASET);
    gc9a01_write_data(x0 >> 8);
    gc9a01_write_data(x0 & 0xFF);
    gc9a01_write_data(x1 >> 8);
    gc9a01_write_data(x1 & 0xFF);
    
    // Row address set
    gc9a01_write_command(GC9A01_RASET);
    gc9a01_write_data(y0 >> 8);
    gc9a01_write_data(y0 & 0xFF);
    gc9a01_write_data(y1 >> 8);
    gc9a01_write_data(y1 & 0xFF);
    
    // Write to RAM
    gc9a01_write_command(GC9A01_RAMWR);
}

bool gc9a01_init(spi_inst_t *spi, uint8_t cs_pin, uint8_t dc_pin, uint8_t rst_pin, 
                 uint8_t bl_pin, uint8_t sck_pin, uint8_t mosi_pin) {
    // Store configuration
    g_lcd.spi_inst = spi;
    g_lcd.cs_pin = cs_pin;
    g_lcd.dc_pin = dc_pin;
    g_lcd.rst_pin = rst_pin;
    g_lcd.bl_pin = bl_pin;
    g_lcd.width = GC9A01_WIDTH;
    g_lcd.height = GC9A01_HEIGHT;
    g_lcd.rotation = 0;
    
    // Initialize SPI
    spi_init(spi, 62500 * 1000); // 62.5MHz
    gpio_set_function(sck_pin, GPIO_FUNC_SPI);
    gpio_set_function(mosi_pin, GPIO_FUNC_SPI);
    
    // Initialize control pins
    gpio_init(cs_pin);
    gpio_set_dir(cs_pin, GPIO_OUT);
    gpio_put(cs_pin, 1);
    
    gpio_init(dc_pin);
    gpio_set_dir(dc_pin, GPIO_OUT);
    
    gpio_init(rst_pin);
    gpio_set_dir(rst_pin, GPIO_OUT);
    
    // Initialize backlight PWM
    gpio_set_function(bl_pin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(bl_pin);
    pwm_set_wrap(slice_num, 255);
    pwm_set_gpio_level(bl_pin, 128); // 50% brightness
    pwm_set_enabled(slice_num, true);
    
    // Hardware reset
    gc9a01_reset_active();
    sleep_ms(100);
    gc9a01_reset_inactive();
    sleep_ms(100);
    
    // Initialization sequence
    gc9a01_write_command(0xEF);
    
    gc9a01_write_command(0xEB);
    gc9a01_write_data(0x14);
    
    gc9a01_write_command(0xFE);
    gc9a01_write_command(0xEF);
    
    gc9a01_write_command(0xEB);
    gc9a01_write_data(0x14);
    
    gc9a01_write_command(0x84);
    gc9a01_write_data(0x40);
    
    gc9a01_write_command(0x85);
    gc9a01_write_data(0xFF);
    
    gc9a01_write_command(0x86);
    gc9a01_write_data(0xFF);
    
    gc9a01_write_command(0x87);
    gc9a01_write_data(0xFF);
    
    gc9a01_write_command(0x88);
    gc9a01_write_data(0x0A);
    
    gc9a01_write_command(0x89);
    gc9a01_write_data(0x21);
    
    gc9a01_write_command(0x8A);
    gc9a01_write_data(0x00);
    
    gc9a01_write_command(0x8B);
    gc9a01_write_data(0x80);
    
    gc9a01_write_command(0x8C);
    gc9a01_write_data(0x01);
    
    gc9a01_write_command(0x8D);
    gc9a01_write_data(0x01);
    
    gc9a01_write_command(0x8E);
    gc9a01_write_data(0xFF);
    
    gc9a01_write_command(0x8F);
    gc9a01_write_data(0xFF);
    
    gc9a01_write_command(0xB6);
    gc9a01_write_data(0x00);
    gc9a01_write_data(0x00);
    
    // Memory Access Control
    gc9a01_write_command(GC9A01_MADCTL);
    gc9a01_write_data(MADCTL_MX | MADCTL_BGR);
    
    // Pixel Format
    gc9a01_write_command(GC9A01_COLMOD);
    gc9a01_write_data(0x55); // 16-bit color
    
    gc9a01_write_command(0x90);
    gc9a01_write_data(0x08);
    gc9a01_write_data(0x08);
    gc9a01_write_data(0x08);
    gc9a01_write_data(0x08);
    
    gc9a01_write_command(0xBD);
    gc9a01_write_data(0x06);
    
    gc9a01_write_command(0xBC);
    gc9a01_write_data(0x00);
    
    gc9a01_write_command(0xFF);
    gc9a01_write_data(0x60);
    gc9a01_write_data(0x01);
    gc9a01_write_data(0x04);
    
    gc9a01_write_command(0xC3);
    gc9a01_write_data(0x13);
    gc9a01_write_command(0xC4);
    gc9a01_write_data(0x13);
    
    gc9a01_write_command(0xC9);
    gc9a01_write_data(0x22);
    
    gc9a01_write_command(0xBE);
    gc9a01_write_data(0x11);
    
    gc9a01_write_command(0xE1);
    gc9a01_write_data(0x10);
    gc9a01_write_data(0x0E);
    
    gc9a01_write_command(0xDF);
    gc9a01_write_data(0x21);
    gc9a01_write_data(0x0C);
    gc9a01_write_data(0x02);
    
    gc9a01_write_command(0xF0);
    gc9a01_write_data(0x45);
    gc9a01_write_data(0x09);
    gc9a01_write_data(0x08);
    gc9a01_write_data(0x08);
    gc9a01_write_data(0x26);
    gc9a01_write_data(0x2A);
    
    gc9a01_write_command(0xF1);
    gc9a01_write_data(0x43);
    gc9a01_write_data(0x70);
    gc9a01_write_data(0x72);
    gc9a01_write_data(0x36);
    gc9a01_write_data(0x37);
    gc9a01_write_data(0x6F);
    
    gc9a01_write_command(0xF2);
    gc9a01_write_data(0x45);
    gc9a01_write_data(0x09);
    gc9a01_write_data(0x08);
    gc9a01_write_data(0x08);
    gc9a01_write_data(0x26);
    gc9a01_write_data(0x2A);
    
    gc9a01_write_command(0xF3);
    gc9a01_write_data(0x43);
    gc9a01_write_data(0x70);
    gc9a01_write_data(0x72);
    gc9a01_write_data(0x36);
    gc9a01_write_data(0x37);
    gc9a01_write_data(0x6F);
    
    gc9a01_write_command(0xED);
    gc9a01_write_data(0x1B);
    gc9a01_write_data(0x0B);
    
    gc9a01_write_command(0xAE);
    gc9a01_write_data(0x77);
    
    gc9a01_write_command(0xCD);
    gc9a01_write_data(0x63);
    
    gc9a01_write_command(0x70);
    gc9a01_write_data(0x07);
    gc9a01_write_data(0x07);
    gc9a01_write_data(0x04);
    gc9a01_write_data(0x0E);
    gc9a01_write_data(0x0F);
    gc9a01_write_data(0x09);
    gc9a01_write_data(0x07);
    gc9a01_write_data(0x08);
    gc9a01_write_data(0x03);
    
    gc9a01_write_command(0xE8);
    gc9a01_write_data(0x34);
    
    gc9a01_write_command(0x62);
    gc9a01_write_data(0x18);
    gc9a01_write_data(0x0D);
    gc9a01_write_data(0x71);
    gc9a01_write_data(0xED);
    gc9a01_write_data(0x70);
    gc9a01_write_data(0x70);
    gc9a01_write_data(0x18);
    gc9a01_write_data(0x0F);
    gc9a01_write_data(0x71);
    gc9a01_write_data(0xEF);
    gc9a01_write_data(0x70);
    gc9a01_write_data(0x70);
    
    gc9a01_write_command(0x63);
    gc9a01_write_data(0x18);
    gc9a01_write_data(0x11);
    gc9a01_write_data(0x71);
    gc9a01_write_data(0xF1);
    gc9a01_write_data(0x70);
    gc9a01_write_data(0x70);
    gc9a01_write_data(0x18);
    gc9a01_write_data(0x13);
    gc9a01_write_data(0x71);
    gc9a01_write_data(0xF3);
    gc9a01_write_data(0x70);
    gc9a01_write_data(0x70);
    
    gc9a01_write_command(0x64);
    gc9a01_write_data(0x28);
    gc9a01_write_data(0x29);
    gc9a01_write_data(0xF1);
    gc9a01_write_data(0x01);
    gc9a01_write_data(0xF1);
    gc9a01_write_data(0x00);
    gc9a01_write_data(0x07);
    
    gc9a01_write_command(0x66);
    gc9a01_write_data(0x3C);
    gc9a01_write_data(0x00);
    gc9a01_write_data(0xCD);
    gc9a01_write_data(0x67);
    gc9a01_write_data(0x45);
    gc9a01_write_data(0x45);
    gc9a01_write_data(0x10);
    gc9a01_write_data(0x00);
    gc9a01_write_data(0x00);
    gc9a01_write_data(0x00);
    
    gc9a01_write_command(0x67);
    gc9a01_write_data(0x00);
    gc9a01_write_data(0x3C);
    gc9a01_write_data(0x00);
    gc9a01_write_data(0x00);
    gc9a01_write_data(0x00);
    gc9a01_write_data(0x01);
    gc9a01_write_data(0x54);
    gc9a01_write_data(0x10);
    gc9a01_write_data(0x32);
    gc9a01_write_data(0x98);
    
    gc9a01_write_command(0x74);
    gc9a01_write_data(0x10);
    gc9a01_write_data(0x85);
    gc9a01_write_data(0x80);
    gc9a01_write_data(0x00);
    gc9a01_write_data(0x00);
    gc9a01_write_data(0x4E);
    gc9a01_write_data(0x00);
    
    gc9a01_write_command(0x98);
    gc9a01_write_data(0x3E);
    gc9a01_write_data(0x07);
    
    gc9a01_write_command(0x35); // Tearing Effect Line ON
    gc9a01_write_command(0x21); // Display Inversion ON
    
    gc9a01_write_command(GC9A01_SLPOUT); // Sleep Out
    sleep_ms(120);
    
    gc9a01_write_command(GC9A01_DISPON); // Display ON
    sleep_ms(20);
    
    // Clear screen
    gc9a01_clear(GC9A01_BLACK);
    
    printf("GC9A01: Initialized successfully\n");
    return true;
}

void gc9a01_clear(uint16_t color) {
    gc9a01_fill_rect(0, 0, g_lcd.width, g_lcd.height, color);
}

void gc9a01_draw_pixel(uint16_t x, uint16_t y, uint16_t color) {
    if (x >= g_lcd.width || y >= g_lcd.height) return;
    
    gc9a01_set_window(x, y, x, y);
    
    uint8_t data[2];
    data[0] = color >> 8;
    data[1] = color & 0xFF;
    
    gc9a01_cs_select();
    gc9a01_dc_data();
    spi_write_blocking(g_lcd.spi_inst, data, 2);
    gc9a01_cs_deselect();
}

void gc9a01_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    if (x >= g_lcd.width || y >= g_lcd.height) return;
    
    if (x + w > g_lcd.width) w = g_lcd.width - x;
    if (y + h > g_lcd.height) h = g_lcd.height - y;
    
    gc9a01_set_window(x, y, x + w - 1, y + h - 1);
    
    uint8_t color_data[2];
    color_data[0] = color >> 8;
    color_data[1] = color & 0xFF;
    
    gc9a01_cs_select();
    gc9a01_dc_data();
    
    for (uint32_t i = 0; i < w * h; i++) {
        spi_write_blocking(g_lcd.spi_inst, color_data, 2);
    }
    
    gc9a01_cs_deselect();
}

void gc9a01_draw_line(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color) {
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    
    while (1) {
        gc9a01_draw_pixel(x0, y0, color);
        
        if (x0 == x1 && y0 == y1) break;
        
        int e2 = 2 * err;
        
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }
}

void gc9a01_draw_circle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color) {
    int x = r;
    int y = 0;
    int err = 0;
    
    while (x >= y) {
        gc9a01_draw_pixel(x0 + x, y0 + y, color);
        gc9a01_draw_pixel(x0 + y, y0 + x, color);
        gc9a01_draw_pixel(x0 - y, y0 + x, color);
        gc9a01_draw_pixel(x0 - x, y0 + y, color);
        gc9a01_draw_pixel(x0 - x, y0 - y, color);
        gc9a01_draw_pixel(x0 - y, y0 - x, color);
        gc9a01_draw_pixel(x0 + y, y0 - x, color);
        gc9a01_draw_pixel(x0 + x, y0 - y, color);
        
        if (err <= 0) {
            y += 1;
            err += 2 * y + 1;
        }
        
        if (err > 0) {
            x -= 1;
            err -= 2 * x + 1;
        }
    }
}

void gc9a01_fill_circle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color) {
    int x = r;
    int y = 0;
    int err = 0;
    
    while (x >= y) {
        gc9a01_draw_line(x0 - x, y0 + y, x0 + x, y0 + y, color);
        gc9a01_draw_line(x0 - y, y0 + x, x0 + y, y0 + x, color);
        gc9a01_draw_line(x0 - x, y0 - y, x0 + x, y0 - y, color);
        gc9a01_draw_line(x0 - y, y0 - x, x0 + y, y0 - x, color);
        
        if (err <= 0) {
            y += 1;
            err += 2 * y + 1;
        }
        
        if (err > 0) {
            x -= 1;
            err -= 2 * x + 1;
        }
    }
}

void gc9a01_draw_image(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *data) {
    if (x >= g_lcd.width || y >= g_lcd.height) return;
    
    if (x + w > g_lcd.width) w = g_lcd.width - x;
    if (y + h > g_lcd.height) h = g_lcd.height - y;
    
    gc9a01_set_window(x, y, x + w - 1, y + h - 1);
    
    gc9a01_cs_select();
    gc9a01_dc_data();
    
    for (uint32_t i = 0; i < w * h; i++) {
        uint8_t pixel[2];
        pixel[0] = data[i] >> 8;
        pixel[1] = data[i] & 0xFF;
        spi_write_blocking(g_lcd.spi_inst, pixel, 2);
    }
    
    gc9a01_cs_deselect();
}

void gc9a01_set_rotation(uint8_t rotation) {
    g_lcd.rotation = rotation % 4;
    
    uint8_t madctl;
    
    switch (g_lcd.rotation) {
        case 0:
            madctl = MADCTL_MX | MADCTL_BGR;
            g_lcd.width = GC9A01_WIDTH;
            g_lcd.height = GC9A01_HEIGHT;
            break;
        case 1:
            madctl = MADCTL_MV | MADCTL_BGR;
            g_lcd.width = GC9A01_HEIGHT;
            g_lcd.height = GC9A01_WIDTH;
            break;
        case 2:
            madctl = MADCTL_MY | MADCTL_BGR;
            g_lcd.width = GC9A01_WIDTH;
            g_lcd.height = GC9A01_HEIGHT;
            break;
        case 3:
            madctl = MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_BGR;
            g_lcd.width = GC9A01_HEIGHT;
            g_lcd.height = GC9A01_WIDTH;
            break;
        default:
            madctl = MADCTL_MX | MADCTL_BGR;
    }
    
    gc9a01_write_command(GC9A01_MADCTL);
    gc9a01_write_data(madctl);
}

void gc9a01_set_backlight(uint8_t brightness) {
    pwm_set_gpio_level(g_lcd.bl_pin, brightness);
}