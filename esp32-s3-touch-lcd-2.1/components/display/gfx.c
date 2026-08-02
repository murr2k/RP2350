#include "gfx.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "lcd_2in1.h"

#define W LCD_2IN1_WIDTH
#define H LCD_2IN1_HEIGHT

#define MAX_PRIMS  384
#define TEXT_POOL  3072

extern const uint8_t gfx_font5x7[96][5];

typedef enum {
    P_RECT = 0,
    P_LINE,
    P_CIRCLE,
    P_DISC,
    P_TEXT,
    P_ROW_FN,
} prim_kind_t;

typedef struct {
    uint8_t kind;
    uint8_t scale;          /* text scale, or line thickness */
    uint16_t color;
    uint16_t bg;            /* text background, GFX_TRANSPARENT for none */
    int16_t y0, y1;         /* inclusive row range, for quick rejection */
    int16_t a, b, c, d;     /* geometry, meaning depends on kind */
    uint16_t text_off;
    uint16_t text_len;
    gfx_row_fn row_fn;
    void *row_ctx;
} prim_t;

static uint16_t *s_fb;
static uint16_t s_bg = GFX_BLACK;
static prim_t s_prims[MAX_PRIMS];
static int s_prim_count;
static char s_text_pool[TEXT_POOL];
static int s_text_used;
static bool s_overflow;
static bool s_flushed;
static uint16_t s_row[W];

/* Half width of the visible circle on each row. */
static int16_t s_span[H];
static bool s_span_ready;

static void build_span_table(void)
{
    const float r = (float)(W / 2);
    for (int y = 0; y < H; y++) {
        const float dy = (float)(y - H / 2) + 0.5f;
        const float d = r * r - dy * dy;
        int half = (d > 0.0f) ? (int)(sqrtf(d) + 1.0f) : 0;
        if (half > W / 2) {
            half = W / 2;
        }
        s_span[y] = (int16_t)half;
    }
    s_span_ready = true;
}

int gfx_visible_half_width(int y)
{
    if (!s_span_ready) {
        build_span_table();
    }
    if (y < 0 || y >= H) {
        return 0;
    }
    return s_span[y];
}

void gfx_bind(uint16_t *fb)
{
    s_fb = fb;
}

uint16_t *gfx_buffer(void)
{
    return s_fb;
}

bool gfx_overflowed(void)
{
    return s_overflow;
}

/* --- recording ------------------------------------------------------------ */

static prim_t *add_prim(prim_kind_t kind, int y0, int y1, uint16_t color)
{
    if (y1 < 0 || y0 >= H) {
        return NULL;            /* entirely off screen */
    }
    if (s_prim_count >= MAX_PRIMS) {
        s_overflow = true;
        return NULL;
    }

    prim_t *p = &s_prims[s_prim_count++];
    memset(p, 0, sizeof(*p));
    p->kind = (uint8_t)kind;
    p->color = color;
    p->bg = GFX_TRANSPARENT;
    p->y0 = (int16_t)((y0 < 0) ? 0 : y0);
    p->y1 = (int16_t)((y1 >= H) ? H - 1 : y1);
    return p;
}

void gfx_clear(uint16_t color)
{
    if (!s_span_ready) {
        build_span_table();
    }
    s_bg = color;
    s_prim_count = 0;
    s_text_used = 0;
    s_overflow = false;
    s_flushed = false;
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
    if (w == 0 || h == 0) {
        return;
    }
    prim_t *p = add_prim(P_RECT, y, y + h - 1, color);
    if (p != NULL) {
        p->a = (int16_t)x;
        p->b = (int16_t)y;
        p->c = (int16_t)w;
        p->d = (int16_t)h;
    }
}

void gfx_pixel(int x, int y, uint16_t color)
{
    gfx_fill_rect(x, y, 1, 1, color);
}

void gfx_hline(int x, int y, int w, uint16_t color)
{
    gfx_fill_rect(x, y, w, 1, color);
}

void gfx_vline(int x, int y, int h, uint16_t color)
{
    gfx_fill_rect(x, y, 1, h, color);
}

void gfx_rect(int x, int y, int w, int h, uint16_t color)
{
    gfx_hline(x, y, w, color);
    gfx_hline(x, y + h - 1, w, color);
    gfx_vline(x, y, h, color);
    gfx_vline(x + w - 1, y, h, color);
}

void gfx_thick_line(int x0, int y0, int x1, int y1, int thickness, uint16_t color)
{
    if (thickness < 1) {
        thickness = 1;
    }
    const int half = thickness / 2;
    const int lo = (y0 < y1) ? y0 : y1;
    const int hi = (y0 < y1) ? y1 : y0;

    prim_t *p = add_prim(P_LINE, lo - half, hi + half, color);
    if (p != NULL) {
        p->scale = (uint8_t)thickness;
        p->a = (int16_t)x0;
        p->b = (int16_t)y0;
        p->c = (int16_t)x1;
        p->d = (int16_t)y1;
    }
}

void gfx_line(int x0, int y0, int x1, int y1, uint16_t color)
{
    gfx_thick_line(x0, y0, x1, y1, 1, color);
}

void gfx_circle(int cx, int cy, int r, uint16_t color)
{
    prim_t *p = add_prim(P_CIRCLE, cy - r, cy + r, color);
    if (p != NULL) {
        p->a = (int16_t)cx;
        p->b = (int16_t)cy;
        p->c = (int16_t)r;
    }
}

void gfx_fill_circle(int cx, int cy, int r, uint16_t color)
{
    prim_t *p = add_prim(P_DISC, cy - r, cy + r, color);
    if (p != NULL) {
        p->a = (int16_t)cx;
        p->b = (int16_t)cy;
        p->c = (int16_t)r;
    }
}

void gfx_arrow(int x0, int y0, int x1, int y1, uint16_t color)
{
    gfx_line(x0, y0, x1, y1, color);

    const float angle = atan2f((float)(y1 - y0), (float)(x1 - x0));
    const float head = 10.0f;
    const float spread = 0.4f;

    gfx_line(x1, y1, x1 - (int)(head * cosf(angle - spread)),
             y1 - (int)(head * sinf(angle - spread)), color);
    gfx_line(x1, y1, x1 - (int)(head * cosf(angle + spread)),
             y1 - (int)(head * sinf(angle + spread)), color);
}

void gfx_row_painter(gfx_row_fn fn, void *ctx)
{
    if (fn == NULL) {
        return;
    }
    prim_t *p = add_prim(P_ROW_FN, 0, H - 1, 0);
    if (p != NULL) {
        p->row_fn = fn;
        p->row_ctx = ctx;
    }
}

/* --- text ----------------------------------------------------------------- */

static void add_text_run(int x, int y, const char *s, int len, uint16_t color,
                         uint16_t bg, int scale)
{
    if (len <= 0) {
        return;
    }
    if (s_text_used + len > TEXT_POOL) {
        s_overflow = true;
        return;
    }

    prim_t *p = add_prim(P_TEXT, y, y + 7 * scale - 1, color);
    if (p == NULL) {
        return;
    }
    p->scale = (uint8_t)scale;
    p->bg = bg;
    p->a = (int16_t)x;
    p->b = (int16_t)y;
    p->text_off = (uint16_t)s_text_used;
    p->text_len = (uint16_t)len;

    memcpy(&s_text_pool[s_text_used], s, (size_t)len);
    s_text_used += len;
}

void gfx_text_bg(int x, int y, const char *s, uint16_t color, uint16_t bg, int scale)
{
    if (scale < 1) {
        scale = 1;
    }
    const char *start = s;
    int len = 0;

    for (;; s++) {
        if (*s == '\0' || *s == '\n') {
            add_text_run(x, y, start, len, color, bg, scale);
            if (*s == '\0') {
                return;
            }
            y += gfx_text_height(scale) + 2 * scale;
            start = s + 1;
            len = 0;
        } else {
            len++;
        }
    }
}

void gfx_text(int x, int y, const char *s, uint16_t color, int scale)
{
    gfx_text_bg(x, y, s, color, GFX_TRANSPARENT, scale);
}

void gfx_char(int x, int y, char c, uint16_t color, uint16_t bg, int scale)
{
    const char s[2] = {c, '\0'};
    gfx_text_bg(x, y, s, color, bg, scale);
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

/* --- rasterising ---------------------------------------------------------- */

static inline void span(int x0, int x1, uint16_t color)
{
    if (x0 < 0) {
        x0 = 0;
    }
    if (x1 > W - 1) {
        x1 = W - 1;
    }
    for (int x = x0; x <= x1; x++) {
        s_row[x] = color;
    }
}

/* One row of a single pixel wide segment. Working from the row's top and bottom
 * edges means consecutive rows always share an x, so shallow lines come out
 * connected rather than dotted. */
static void raster_segment(int x0, int y0, int x1, int y1, int y, uint16_t color)
{
    if (y0 == y1) {
        if (y == y0) {
            span((x0 < x1) ? x0 : x1, (x0 < x1) ? x1 : x0, color);
        }
        return;
    }

    const int lo = (y0 < y1) ? y0 : y1;
    const int hi = (y0 < y1) ? y1 : y0;
    if (y < lo || y > hi) {
        return;
    }

    const float slope = (float)(x1 - x0) / (float)(y1 - y0);
    float ya = (float)y - 0.5f;
    float yb = (float)y + 0.5f;
    if (ya < (float)lo) {
        ya = (float)lo;
    }
    if (yb > (float)hi) {
        yb = (float)hi;
    }

    const float xa = (float)x0 + (ya - (float)y0) * slope;
    const float xb = (float)x0 + (yb - (float)y0) * slope;
    const int ia = (int)lroundf((xa < xb) ? xa : xb);
    const int ib = (int)lroundf((xa < xb) ? xb : xa);
    span(ia, ib, color);
}

static void raster_line(const prim_t *p, int y)
{
    const int thickness = (p->scale < 1) ? 1 : p->scale;
    const int half = thickness / 2;
    const bool horizontal = (abs(p->c - p->a) >= abs(p->d - p->b));

    for (int k = -half; k <= half; k++) {
        if (horizontal) {
            raster_segment(p->a, p->b + k, p->c, p->d + k, y, p->color);
        } else {
            raster_segment(p->a + k, p->b, p->c + k, p->d, y, p->color);
        }
    }
}

static void raster_circle(const prim_t *p, int y)
{
    const float dy = (float)(y - p->b);
    const float outer = (float)p->c + 0.5f;
    const float inner = (float)p->c - 0.5f;

    const float o2 = outer * outer - dy * dy;
    if (o2 < 0.0f) {
        return;
    }
    const int xo = (int)sqrtf(o2);
    const float i2 = inner * inner - dy * dy;

    if (i2 <= 0.0f) {
        span(p->a - xo, p->a + xo, p->color);    /* the caps */
    } else {
        const int xi = (int)sqrtf(i2);
        span(p->a - xo, p->a - xi, p->color);
        span(p->a + xi, p->a + xo, p->color);
    }
}

static void raster_disc(const prim_t *p, int y)
{
    const float dy = (float)(y - p->b);
    const float d2 = (float)p->c * (float)p->c - dy * dy;
    if (d2 < 0.0f) {
        return;
    }
    const int dx = (int)sqrtf(d2);
    span(p->a - dx, p->a + dx, p->color);
}

static void raster_text(const prim_t *p, int y)
{
    const int scale = (p->scale < 1) ? 1 : p->scale;
    const int glyph_row = (y - p->b) / scale;
    if (glyph_row < 0 || glyph_row > 6) {
        return;
    }

    for (int i = 0; i < p->text_len; i++) {
        unsigned char ch = (unsigned char)s_text_pool[p->text_off + i];
        if (ch < 0x20 || ch > 0x7F) {
            ch = '?';
        }
        const uint8_t *glyph = gfx_font5x7[ch - 0x20];
        const int gx = p->a + i * 6 * scale;

        for (int col = 0; col < 5; col++) {
            const int x = gx + col * scale;
            if ((glyph[col] >> glyph_row) & 0x01u) {
                span(x, x + scale - 1, p->color);
            } else if (p->bg != GFX_TRANSPARENT) {
                span(x, x + scale - 1, p->bg);
            }
        }
        if (p->bg != GFX_TRANSPARENT) {
            span(gx + 5 * scale, gx + 6 * scale - 1, p->bg);
        }
    }
}

static void fill_background(int x_lo, int count)
{
    if ((s_bg >> 8) == (s_bg & 0xFF)) {
        memset(&s_row[x_lo], s_bg & 0xFF, (size_t)count * sizeof(uint16_t));
        return;
    }
    for (int i = 0; i < count; i++) {
        s_row[x_lo + i] = s_bg;
    }
}

void gfx_flush(void)
{
    if (s_fb == NULL || s_flushed) {
        return;
    }
    if (!s_span_ready) {
        build_span_table();
    }

    for (int y = 0; y < H; y++) {
        const int half = s_span[y];
        if (half == 0) {
            continue;           /* no pixels behind this row of the glass */
        }
        const int x_lo = W / 2 - half;
        const int count = half * 2;

        fill_background(x_lo, count);

        for (int i = 0; i < s_prim_count; i++) {
            const prim_t *p = &s_prims[i];
            if (y < p->y0 || y > p->y1) {
                continue;
            }
            switch ((prim_kind_t)p->kind) {
            case P_RECT:
                span(p->a, p->a + p->c - 1, p->color);
                break;
            case P_LINE:
                raster_line(p, y);
                break;
            case P_CIRCLE:
                raster_circle(p, y);
                break;
            case P_DISC:
                raster_disc(p, y);
                break;
            case P_TEXT:
                raster_text(p, y);
                break;
            case P_ROW_FN:
                p->row_fn(y, s_row, p->row_ctx);
                break;
            }
        }

        memcpy(&s_fb[y * W + x_lo], &s_row[x_lo], (size_t)count * sizeof(uint16_t));
    }

    s_flushed = true;
}
