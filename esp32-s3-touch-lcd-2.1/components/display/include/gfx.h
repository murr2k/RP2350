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

/** Scale a colour's brightness by num/den, for fading things out with distance
 *  on a screen that has no alpha channel. */
uint16_t gfx_dim(uint16_t color, int num, int den);

/** Pass as the background colour to leave the existing pixels alone. */
#define GFX_TRANSPARENT 0x0001

/** Printable character that renders as a degree sign. */
#define GFX_DEG "\x7f"

/* Drawing is recorded, not painted, and there is no frame buffer.
 *
 * Every call below appends to a display list. The panel's interrupt then asks
 * for a handful of rows at a time, as its DMA needs them, and those rows are
 * composed straight into the bounce buffer in internal SRAM: background first,
 * then each primitive that crosses the row, in order.
 *
 * Nothing is ever stored at frame size. The 450 KB frame buffers are gone, and
 * with them the writes that filled them and the copy that read them back, which
 * is the traffic the whole frame budget used to be spent on.
 *
 * The drawing API is unchanged, so demos need not care about any of this. */

/** Retained for source compatibility. There is no frame buffer to bind. */
void gfx_bind(uint16_t *fb);

/** Always NULL now: nothing holds a whole frame. */
uint16_t *gfx_buffer(void);

/** True if the list being recorded overflowed and primitives were dropped. */
bool gfx_overflowed(void);

/** Paint a row yourself, for content no primitive describes, such as a per
 *  pixel gradient. Called while composing, in list order like any other
 *  primitive, from interrupt context. Keep it short and self contained. */
typedef void (*gfx_row_fn)(int y, uint16_t *row, void *ctx);
void gfx_row_painter(gfx_row_fn fn, void *ctx);

/* --- frame lifecycle, driven by the display component --------------------- */

/** Point recording at whichever list the panel is not reading. */
void gfx_begin_frame(void);

/** Offer the recorded list to the panel; it is adopted at the next frame. */
void gfx_commit(void);

/** Adopt a committed list. Called from the panel's frame boundary. */
void gfx_swap_lists(void);

/** Compose rows [first_row, first_row + row_count) of the active list into
 *  dest, which holds row_count consecutive rows of full width pixels. Called
 *  from the panel's interrupt. */
void gfx_compose_rows(uint16_t *dest, int first_row, int row_count);

/** Half width of the visible circle on row y. The panel is round, so anything
 *  drawn further than this from the centre column is invisible. */
int gfx_visible_half_width(int y);

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
