#ifndef DISPLAY_DRIVER_H
#define DISPLAY_DRIVER_H

#include "system_types.h"
#include "board_config.h"

// Display orientation
typedef enum {
    DISPLAY_ORIENTATION_0 = 0,
    DISPLAY_ORIENTATION_90 = 1,
    DISPLAY_ORIENTATION_180 = 2,
    DISPLAY_ORIENTATION_270 = 3
} display_orientation_t;

// Font sizes
typedef enum {
    FONT_SIZE_8 = 8,
    FONT_SIZE_12 = 12,
    FONT_SIZE_16 = 16,
    FONT_SIZE_20 = 20,
    FONT_SIZE_24 = 24
} font_size_t;

/**
 * @brief Initialize the display driver
 * @return SYSTEM_OK on success, error code on failure
 */
system_result_t display_init(void);

/**
 * @brief Set display brightness
 * @param brightness Brightness level (0-255)
 * @return SYSTEM_OK on success, error code on failure
 */
system_result_t display_set_brightness(uint8_t brightness);

/**
 * @brief Get current display brightness
 * @return Current brightness level (0-255)
 */
uint8_t display_get_brightness(void);

/**
 * @brief Set display orientation
 * @param orientation Display orientation
 * @return SYSTEM_OK on success, error code on failure
 */
system_result_t display_set_orientation(display_orientation_t orientation);

/**
 * @brief Clear the entire display with specified color
 * @param color RGB565 color value
 */
void display_clear(color565_t color);

/**
 * @brief Set a single pixel
 * @param x X coordinate
 * @param y Y coordinate  
 * @param color RGB565 color value
 */
void display_set_pixel(uint16_t x, uint16_t y, color565_t color);

/**
 * @brief Draw a line
 * @param x0 Start X coordinate
 * @param y0 Start Y coordinate
 * @param x1 End X coordinate
 * @param y1 End Y coordinate
 * @param color RGB565 color value
 */
void display_draw_line(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, color565_t color);

/**
 * @brief Draw a rectangle outline
 * @param x X coordinate of top-left corner
 * @param y Y coordinate of top-left corner
 * @param width Rectangle width
 * @param height Rectangle height
 * @param color RGB565 color value
 */
void display_draw_rect(uint16_t x, uint16_t y, uint16_t width, uint16_t height, color565_t color);

/**
 * @brief Draw a filled rectangle
 * @param x X coordinate of top-left corner
 * @param y Y coordinate of top-left corner
 * @param width Rectangle width
 * @param height Rectangle height
 * @param color RGB565 color value
 */
void display_fill_rect(uint16_t x, uint16_t y, uint16_t width, uint16_t height, color565_t color);

/**
 * @brief Draw a circle outline
 * @param x Center X coordinate
 * @param y Center Y coordinate
 * @param radius Circle radius
 * @param color RGB565 color value
 */
void display_draw_circle(uint16_t x, uint16_t y, uint16_t radius, color565_t color);

/**
 * @brief Draw a filled circle
 * @param x Center X coordinate
 * @param y Center Y coordinate
 * @param radius Circle radius
 * @param color RGB565 color value
 */
void display_fill_circle(uint16_t x, uint16_t y, uint16_t radius, color565_t color);

/**
 * @brief Draw text
 * @param text Text string to draw
 * @param x X coordinate of text start
 * @param y Y coordinate of text start
 * @param color RGB565 text color
 * @param bg_color RGB565 background color
 * @param font_size Font size
 * @return Width of drawn text in pixels
 */
uint16_t display_draw_text(const char *text, uint16_t x, uint16_t y, 
                          color565_t color, color565_t bg_color, font_size_t font_size);

/**
 * @brief Get text width in pixels
 * @param text Text string
 * @param font_size Font size
 * @return Text width in pixels
 */
uint16_t display_get_text_width(const char *text, font_size_t font_size);

/**
 * @brief Draw a bitmap image
 * @param x X coordinate
 * @param y Y coordinate
 * @param width Image width
 * @param height Image height
 * @param bitmap Bitmap data (RGB565)
 */
void display_draw_bitmap(uint16_t x, uint16_t y, uint16_t width, uint16_t height, 
                        const color565_t *bitmap);

/**
 * @brief Update the display (if using framebuffer)
 */
void display_update(void);

/**
 * @brief Put display into sleep mode
 * @return SYSTEM_OK on success, error code on failure
 */
system_result_t display_sleep(void);

/**
 * @brief Wake display from sleep mode
 * @return SYSTEM_OK on success, error code on failure
 */
system_result_t display_wake(void);

#endif // DISPLAY_DRIVER_H