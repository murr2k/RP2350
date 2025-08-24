/**
 * GC9A01 Round LCD Display Driver Header
 * For Waveshare RP2350-LCD-1.28 (240x240 pixels)
 */

#ifndef GC9A01_H
#define GC9A01_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/spi.h"

// Display dimensions
#define GC9A01_WIDTH    240
#define GC9A01_HEIGHT   240

// Color definitions (RGB565)
#define GC9A01_BLACK    0x0000
#define GC9A01_WHITE    0xFFFF
#define GC9A01_RED      0xF800
#define GC9A01_GREEN    0x07E0
#define GC9A01_BLUE     0x001F
#define GC9A01_CYAN     0x07FF
#define GC9A01_MAGENTA  0xF81F
#define GC9A01_YELLOW   0xFFE0
#define GC9A01_ORANGE   0xFD20
#define GC9A01_PURPLE   0x8010
#define GC9A01_GRAY     0x8410

// Color conversion macros
#define RGB565(r, g, b) (((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3))
#define RGB888_TO_RGB565(r, g, b) (((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3))

// Display structure
typedef struct {
    spi_inst_t *spi_inst;
    uint8_t cs_pin;
    uint8_t dc_pin;
    uint8_t rst_pin;
    uint8_t bl_pin;
    uint16_t width;
    uint16_t height;
    uint8_t rotation;
} gc9a01_t;

/**
 * Initialize the GC9A01 display
 * @param spi SPI instance (spi0 or spi1)
 * @param cs_pin Chip select GPIO pin
 * @param dc_pin Data/Command GPIO pin
 * @param rst_pin Reset GPIO pin
 * @param bl_pin Backlight GPIO pin
 * @param sck_pin SPI clock GPIO pin
 * @param mosi_pin SPI MOSI GPIO pin
 * @return true if successful, false otherwise
 */
bool gc9a01_init(spi_inst_t *spi, uint8_t cs_pin, uint8_t dc_pin, uint8_t rst_pin,
                 uint8_t bl_pin, uint8_t sck_pin, uint8_t mosi_pin);

/**
 * Clear the entire display with a color
 * @param color RGB565 color value
 */
void gc9a01_clear(uint16_t color);

/**
 * Draw a single pixel
 * @param x X coordinate
 * @param y Y coordinate
 * @param color RGB565 color value
 */
void gc9a01_draw_pixel(uint16_t x, uint16_t y, uint16_t color);

/**
 * Fill a rectangle with a color
 * @param x X coordinate of top-left corner
 * @param y Y coordinate of top-left corner
 * @param w Width in pixels
 * @param h Height in pixels
 * @param color RGB565 color value
 */
void gc9a01_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);

/**
 * Draw a line between two points
 * @param x0 Start X coordinate
 * @param y0 Start Y coordinate
 * @param x1 End X coordinate
 * @param y1 End Y coordinate
 * @param color RGB565 color value
 */
void gc9a01_draw_line(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);

/**
 * Draw a circle outline
 * @param x0 Center X coordinate
 * @param y0 Center Y coordinate
 * @param r Radius in pixels
 * @param color RGB565 color value
 */
void gc9a01_draw_circle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color);

/**
 * Draw a filled circle
 * @param x0 Center X coordinate
 * @param y0 Center Y coordinate
 * @param r Radius in pixels
 * @param color RGB565 color value
 */
void gc9a01_fill_circle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color);

/**
 * Draw an image from RGB565 data
 * @param x X coordinate of top-left corner
 * @param y Y coordinate of top-left corner
 * @param w Width in pixels
 * @param h Height in pixels
 * @param data Pointer to RGB565 image data
 */
void gc9a01_draw_image(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *data);

/**
 * Set display rotation
 * @param rotation Rotation value (0-3)
 * 0 = 0 degrees, 1 = 90 degrees, 2 = 180 degrees, 3 = 270 degrees
 */
void gc9a01_set_rotation(uint8_t rotation);

/**
 * Set backlight brightness
 * @param brightness Brightness level (0-255)
 */
void gc9a01_set_backlight(uint8_t brightness);

#endif // GC9A01_H