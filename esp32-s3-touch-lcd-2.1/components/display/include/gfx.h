/**
 * Small RGB565 drawing helpers shared by every demo.
 *
 * The RP2350 demos each carried their own copy of set_pixel / draw_line /
 * clear_buffer, all writing into a private 240x240 array. Here the frame buffer
 * belongs to the display driver, so a demo binds the buffer it got from
 * LCD_2IN1_GetBuffer() once per frame and then draws with the same calls.
 *
 * The text routines are new: the 1.28" demos had no font, which is why several
 * of them fall back to drawing bar graphs where a number would be clearer.
 */

#ifndef GFX_H
#define GFX_H

#include <stdbool.h>
#include <stdint.h>

/* RGB565 colours, matching the constants used by the RP2350 demos. */
#define GFX_BLACK   0x0000
#define GFX_WHITE   0xFFFF
#define GFX_RED     0xF800
#define GFX_GREEN   0x07E0
#define GFX_BLUE    0x001F
#define GFX_CYAN    0x07FF
#define GFX_MAGENTA 0xF81F
#define GFX_YELLOW  0xFFE0
#define GFX_ORANGE  0xFD20
#define GFX_GREY    0x8410
#define GFX_DGREY   0x4208
#define GFX_DGREEN  0x0410

#define GFX_RGB(r, g, b) \
    ((uint16_t)((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | (((b) & 0xF8) >> 3)))

/** Pass as the background colour to leave the existing pixels alone. */
#define GFX_TRANSPARENT 0x0001

/** Printable character that renders as a degree sign. */
#define GFX_DEG "\x7f"

/** Point every following call at this frame buffer. */
void gfx_bind(uint16_t *fb);
uint16_t *gfx_buffer(void);

void gfx_clear(uint16_t color);
void gfx_pixel(int x, int y, uint16_t color);
void gfx_hline(int x, int y, int w, uint16_t color);
void gfx_vline(int x, int y, int h, uint16_t color);
void gfx_line(int x0, int y0, int x1, int y1, uint16_t color);
void gfx_thick_line(int x0, int y0, int x1, int y1, int thickness, uint16_t color);
void gfx_rect(int x, int y, int w, int h, uint16_t color);
void gfx_fill_rect(int x, int y, int w, int h, uint16_t color);
void gfx_circle(int cx, int cy, int r, uint16_t color);
void gfx_fill_circle(int cx, int cy, int r, uint16_t color);

/** Arrow head at (x1,y1) pointing away from (x0,y0). */
void gfx_arrow(int x0, int y0, int x1, int y1, uint16_t color);

void gfx_char(int x, int y, char c, uint16_t color, uint16_t bg, int scale);
void gfx_text(int x, int y, const char *s, uint16_t color, int scale);
void gfx_text_bg(int x, int y, const char *s, uint16_t color, uint16_t bg, int scale);
void gfx_text_centered(int cx, int y, const char *s, uint16_t color, int scale);
void gfx_printf(int x, int y, uint16_t color, int scale, const char *fmt, ...)
    __attribute__((format(printf, 5, 6)));

int gfx_text_width(const char *s, int scale);
int gfx_text_height(int scale);

#endif /* GFX_H */
