#include "gfx.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "lcd_2in1.h"

#define W LCD_2IN1_WIDTH
#define H LCD_2IN1_HEIGHT

extern const uint8_t gfx_font5x7[96][5];

static uint16_t *s_fb;

void gfx_bind(uint16_t *fb)
{
    s_fb = fb;
}

uint16_t *gfx_buffer(void)
{
    return s_fb;
}

void gfx_clear(uint16_t color)
{
    if (s_fb == NULL) {
        return;
    }
    if ((color >> 8) == (color & 0xFF)) {
        memset(s_fb, color & 0xFF, (size_t)W * H * sizeof(uint16_t));
        return;
    }
    /* Writing 32 bits at a time roughly halves the PSRAM traffic. */
    const uint32_t pair = ((uint32_t)color << 16) | color;
    uint32_t *p = (uint32_t *)s_fb;
    for (int i = 0; i < W * H / 2; i++) {
        p[i] = pair;
    }
}

void gfx_pixel(int x, int y, uint16_t color)
{
    if (s_fb != NULL && x >= 0 && x < W && y >= 0 && y < H) {
        s_fb[y * W + x] = color;
    }
}

void gfx_hline(int x, int y, int w, uint16_t color)
{
    if (w < 0) {
        x += w;
        w = -w;
    }
    for (int i = 0; i < w; i++) {
        gfx_pixel(x + i, y, color);
    }
}

void gfx_vline(int x, int y, int h, uint16_t color)
{
    if (h < 0) {
        y += h;
        h = -h;
    }
    for (int i = 0; i < h; i++) {
        gfx_pixel(x, y + i, color);
    }
}

void gfx_line(int x0, int y0, int x1, int y1, uint16_t color)
{
    /* Same Bresenham loop the RP2350 demos used. */
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    for (;;) {
        gfx_pixel(x0, y0, color);
        if (x0 == x1 && y0 == y1) {
            break;
        }
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

void gfx_thick_line(int x0, int y0, int x1, int y1, int thickness, uint16_t color)
{
    if (thickness < 1) {
        thickness = 1;
    }
    const int half = thickness / 2;
    /* Offset perpendicular to the dominant axis; good enough for wireframes. */
    if (abs(x1 - x0) >= abs(y1 - y0)) {
        for (int i = -half; i <= half; i++) {
            gfx_line(x0, y0 + i, x1, y1 + i, color);
        }
    } else {
        for (int i = -half; i <= half; i++) {
            gfx_line(x0 + i, y0, x1 + i, y1, color);
        }
    }
}

void gfx_rect(int x, int y, int w, int h, uint16_t color)
{
    gfx_hline(x, y, w, color);
    gfx_hline(x, y + h - 1, w, color);
    gfx_vline(x, y, h, color);
    gfx_vline(x + w - 1, y, h, color);
}

void gfx_fill_rect(int x, int y, int w, int h, uint16_t color)
{
    if (w < 0) {
        x += w;
        w = -w;
    }
    if (h < 0) {
        y += h;
        h = -h;
    }
    for (int row = 0; row < h; row++) {
        gfx_hline(x, y + row, w, color);
    }
}

void gfx_circle(int cx, int cy, int r, uint16_t color)
{
    int x = r;
    int y = 0;
    int err = 1 - r;

    while (x >= y) {
        gfx_pixel(cx + x, cy + y, color);
        gfx_pixel(cx + y, cy + x, color);
        gfx_pixel(cx - y, cy + x, color);
        gfx_pixel(cx - x, cy + y, color);
        gfx_pixel(cx - x, cy - y, color);
        gfx_pixel(cx - y, cy - x, color);
        gfx_pixel(cx + y, cy - x, color);
        gfx_pixel(cx + x, cy - y, color);
        y++;
        if (err < 0) {
            err += 2 * y + 1;
        } else {
            x--;
            err += 2 * (y - x) + 1;
        }
    }
}

void gfx_fill_circle(int cx, int cy, int r, uint16_t color)
{
    for (int dy = -r; dy <= r; dy++) {
        const int dx = (int)sqrtf((float)(r * r - dy * dy));
        gfx_hline(cx - dx, cy + dy, 2 * dx + 1, color);
    }
}

void gfx_arrow(int x0, int y0, int x1, int y1, uint16_t color)
{
    gfx_line(x0, y0, x1, y1, color);

    const float angle = atan2f((float)(y1 - y0), (float)(x1 - x0));
    const float head = 10.0f;
    const float spread = 0.4f;

    const int ax1 = x1 - (int)(head * cosf(angle - spread));
    const int ay1 = y1 - (int)(head * sinf(angle - spread));
    const int ax2 = x1 - (int)(head * cosf(angle + spread));
    const int ay2 = y1 - (int)(head * sinf(angle + spread));

    gfx_line(x1, y1, ax1, ay1, color);
    gfx_line(x1, y1, ax2, ay2, color);
}

/* --- text ----------------------------------------------------------------- */

void gfx_char(int x, int y, char c, uint16_t color, uint16_t bg, int scale)
{
    if (scale < 1) {
        scale = 1;
    }
    unsigned char ch = (unsigned char)c;
    if (ch < 0x20 || ch > 0x7F) {
        ch = '?';
    }
    const uint8_t *glyph = gfx_font5x7[ch - 0x20];

    for (int col = 0; col < 6; col++) {
        const uint8_t bits = (col < 5) ? glyph[col] : 0x00; /* trailing spacer */
        for (int row = 0; row < 7; row++) {
            const bool on = (bits >> row) & 0x01;
            if (!on && bg == GFX_TRANSPARENT) {
                continue;
            }
            const uint16_t pixel = on ? color : bg;
            if (scale == 1) {
                gfx_pixel(x + col, y + row, pixel);
            } else {
                gfx_fill_rect(x + col * scale, y + row * scale, scale, scale, pixel);
            }
        }
    }
}

void gfx_text_bg(int x, int y, const char *s, uint16_t color, uint16_t bg, int scale)
{
    if (scale < 1) {
        scale = 1;
    }
    int cursor = x;
    for (; *s; s++) {
        if (*s == '\n') {
            y += gfx_text_height(scale) + 2 * scale;
            cursor = x;
            continue;
        }
        gfx_char(cursor, y, *s, color, bg, scale);
        cursor += 6 * scale;
    }
}

void gfx_text(int x, int y, const char *s, uint16_t color, int scale)
{
    gfx_text_bg(x, y, s, color, GFX_TRANSPARENT, scale);
}

void gfx_text_centered(int cx, int y, const char *s, uint16_t color, int scale)
{
    gfx_text(cx - gfx_text_width(s, scale) / 2, y, s, color, scale);
}

void gfx_printf(int x, int y, uint16_t color, int scale, const char *fmt, ...)
{
    char buf[160];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    gfx_text(x, y, buf, color, scale);
}

int gfx_text_width(const char *s, int scale)
{
    if (scale < 1) {
        scale = 1;
    }
    int longest = 0;
    int width = 0;
    for (; *s; s++) {
        if (*s == '\n') {
            if (width > longest) {
                longest = width;
            }
            width = 0;
            continue;
        }
        width += 6 * scale;
    }
    return (width > longest) ? width : longest;
}

int gfx_text_height(int scale)
{
    return 7 * ((scale < 1) ? 1 : scale);
}
