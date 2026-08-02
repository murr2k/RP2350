#include "gfx.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_timer.h"
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
    P_RING,
    P_TEXT,
    P_ROW_FN,
} prim_kind_t;

typedef struct {
    uint8_t kind;
    uint8_t additive;       /* add into the row instead of overwriting */
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

typedef struct {
    prim_t prims[MAX_PRIMS];
    int count;
    char text[TEXT_POOL];
    int text_used;
    uint16_t bg;
    bool overflow;
} display_list_t;

/* Two lists. The render task fills one while the panel's interrupt composes the
 * other, and they change places at a frame boundary. This is the double
 * buffering that used to cost two 450 KB frames in PSRAM, for 15 KB of SRAM. */
static display_list_t s_lists[2];
static display_list_t *s_record = &s_lists[0];
static display_list_t *volatile s_active = &s_lists[1];
static display_list_t *volatile s_pending;

/* Row being composed. Composition is single threaded inside the ISR, so a
 * single target pointer is enough and keeps the rasterisers simple. */
static uint16_t *s_target;

/* Half width of the visible circle on each row, for callers placing content. */
static int16_t s_span[H];
static bool s_span_ready;

/* Rows given up on to make the deadline. Written in the interrupt, read by
 * whoever is watching the frame budget. */
static volatile uint32_t s_trimmed_rows;

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

uint16_t gfx_dim(uint16_t color, int num, int den)
{
    if (den <= 0) {
        return color;
    }
    if (num < 0) {
        num = 0;
    }
    if (num > den) {
        num = den;
    }

    const int r = ((color >> 11) & 0x1F) * num / den;
    const int g = ((color >> 5) & 0x3F) * num / den;
    const int b = (color & 0x1F) * num / den;
    return (uint16_t)((r << 11) | (g << 5) | b);
}

void gfx_bind(uint16_t *fb)
{
    (void)fb;   /* there is no frame buffer any more */
}

uint16_t *gfx_buffer(void)
{
    return NULL;
}

bool gfx_overflowed(void)
{
    return s_record->overflow;
}

/* --- recording ------------------------------------------------------------ */

static prim_t *add_prim(prim_kind_t kind, int y0, int y1, uint16_t color)
{
    display_list_t *list = s_record;

    if (y1 < 0 || y0 >= H) {
        return NULL;            /* entirely off screen */
    }
    if (list->count >= MAX_PRIMS) {
        list->overflow = true;
        return NULL;
    }

    prim_t *p = &list->prims[list->count++];
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
    s_record->bg = color;
    s_record->count = 0;
    s_record->text_used = 0;
    s_record->overflow = false;
}

void gfx_commit(void)
{
    s_pending = s_record;
}

void gfx_swap_lists(void)
{
    display_list_t *pending = s_pending;
    if (pending != NULL) {
        s_active = pending;
        s_pending = NULL;
    }
}

void gfx_begin_frame(void)
{
    /* Record into whichever list the panel is not reading. */
    s_record = (s_active == &s_lists[0]) ? &s_lists[1] : &s_lists[0];
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

static void ring(int cx, int cy, int r_inner, int r_outer, uint16_t color, bool additive)
{
    if (r_outer <= 0) {
        return;
    }
    if (r_inner < 0) {
        r_inner = 0;
    }
    if (r_inner > r_outer) {
        return;
    }

    prim_t *p = add_prim(P_RING, cy - r_outer, cy + r_outer, color);
    if (p != NULL) {
        p->a = (int16_t)cx;
        p->b = (int16_t)cy;
        p->c = (int16_t)r_inner;
        p->d = (int16_t)r_outer;
        p->additive = additive ? 1u : 0u;
    }
}

void gfx_ring(int cx, int cy, int r_inner, int r_outer, uint16_t color)
{
    ring(cx, cy, r_inner, r_outer, color, false);
}

void gfx_ring_add(int cx, int cy, int r_inner, int r_outer, uint16_t color)
{
    ring(cx, cy, r_inner, r_outer, color, true);
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
    display_list_t *list = s_record;

    if (len <= 0) {
        return;
    }
    if (list->text_used + len > TEXT_POOL) {
        list->overflow = true;
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
    p->text_off = (uint16_t)list->text_used;
    p->text_len = (uint16_t)len;

    memcpy(&list->text[list->text_used], s, (size_t)len);
    list->text_used += len;
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

/* Everything below runs in the panel's interrupt, and Xtensa forbids the
 * floating point unit there: touching a float in an ISR raises a coprocessor
 * exception and panics the core. (CONFIG_FREERTOS_FPU_IN_ISR exists but is
 * ESP32 only, not S3.) So the rasterisers are integer throughout. Recording,
 * which runs in a task, is free to use floats and does. */

/** Integer square root, rounded down. */
static inline uint32_t isqrt32(uint32_t v)
{
    uint32_t rem = 0;
    uint32_t root = 0;

    for (int i = 0; i < 16; i++) {
        root <<= 1;
        rem = (rem << 2) | (v >> 30);
        v <<= 2;
        if (root < rem) {
            rem -= root | 1u;
            root += 2;
        }
    }
    return root >> 1;
}

/** Division rounded to nearest, correct for negative numerators. */
static inline int div_round(int num, int den)
{
    if (den < 0) {
        num = -num;
        den = -den;
    }
    return (num >= 0) ? (num + den / 2) / den : -(((-num) + den / 2) / den);
}

static inline void span(int x0, int x1, uint16_t color)
{
    if (x0 < 0) {
        x0 = 0;
    }
    if (x1 > W - 1) {
        x1 = W - 1;
    }
    for (int x = x0; x <= x1; x++) {
        s_target[x] = color;
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

    /* Half rows, so the row's top and bottom edges land exactly on integers
     * and no floating point is needed. */
    const int dx = x1 - x0;
    const int dy = y1 - y0;
    int ya2 = 2 * y - 1;
    int yb2 = 2 * y + 1;
    if (ya2 < 2 * lo) {
        ya2 = 2 * lo;
    }
    if (yb2 > 2 * hi) {
        yb2 = 2 * hi;
    }

    const int xa = x0 + div_round((ya2 - 2 * y0) * dx, 2 * dy);
    const int xb = x0 + div_round((yb2 - 2 * y0) * dx, 2 * dy);
    span((xa < xb) ? xa : xb, (xa < xb) ? xb : xa, color);
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
    /* Doubled units, so the half pixel band either side of the radius is exact
     * in integers: outer is 2r+1, inner 2r-1, and the square root comes back
     * doubled too. */
    const int dy2 = 2 * (y - p->b);
    const int outer = 2 * p->c + 1;
    const int inner = 2 * p->c - 1;

    const int o2 = outer * outer - dy2 * dy2;
    if (o2 < 0) {
        return;
    }
    const int xo = (int)isqrt32((uint32_t)o2) / 2;
    const int i2 = (inner > 0) ? (inner * inner - dy2 * dy2) : -1;

    if (i2 <= 0) {
        span(p->a - xo, p->a + xo, p->color);    /* the caps */
    } else {
        const int xi = (int)isqrt32((uint32_t)i2) / 2;
        span(p->a - xo, p->a - xi, p->color);
        span(p->a + xi, p->a + xo, p->color);
    }
}

/* Saturating add in RGB565, so overlapping ripples brighten rather than the
 * last one drawn winning. */
static inline void span_add(int x0, int x1, uint16_t color)
{
    if (x0 < 0) {
        x0 = 0;
    }
    if (x1 > W - 1) {
        x1 = W - 1;
    }

    /* Red and blue are added together in one word and green in another, since
     * neither group can carry into the other's bits. That is three adds and
     * three rarely taken saturation fixups per pixel, instead of unpacking and
     * repacking all three channels. This runs on every pixel of every ring on
     * every row, so the difference is the demo keeping its deadline or not. */
    const uint32_t crb = (uint32_t)color & 0xF81Fu;
    const uint32_t cg = (uint32_t)color & 0x07E0u;
    if ((crb | cg) == 0u) {
        return;
    }

    for (int x = x0; x <= x1; x++) {
        const uint32_t d = s_target[x];
        uint32_t rb = (d & 0xF81Fu) + crb;
        uint32_t g = (d & 0x07E0u) + cg;

        if (rb & 0x00020u) {
            rb |= 0x0001Fu;         /* blue overflowed its five bits */
        }
        if (rb & 0x10000u) {
            rb |= 0x0F800u;         /* red overflowed */
        }
        if (g & 0x00800u) {
            g |= 0x007E0u;          /* green overflowed its six */
        }
        s_target[x] = (uint16_t)((rb & 0xF81Fu) | (g & 0x07E0u));
    }
}

static void raster_ring(const prim_t *p, int y)
{
    const int dy = y - p->b;
    const int o2 = p->d * p->d - dy * dy;
    if (o2 < 0) {
        return;
    }
    const int xo = (int)isqrt32((uint32_t)o2);
    const int i2 = p->c * p->c - dy * dy;

    if (i2 <= 0) {
        if (p->additive) {
            span_add(p->a - xo, p->a + xo, p->color);
        } else {
            span(p->a - xo, p->a + xo, p->color);
        }
        return;
    }

    const int xi = (int)isqrt32((uint32_t)i2);
    if (p->additive) {
        span_add(p->a - xo, p->a - xi, p->color);
        span_add(p->a + xi, p->a + xo, p->color);
    } else {
        span(p->a - xo, p->a - xi, p->color);
        span(p->a + xi, p->a + xo, p->color);
    }
}

static void raster_disc(const prim_t *p, int y)
{
    const int dy = y - p->b;
    const int d2 = p->c * p->c - dy * dy;
    if (d2 < 0) {
        return;
    }
    const int dx = (int)isqrt32((uint32_t)d2);
    span(p->a - dx, p->a + dx, p->color);
}

static void raster_text(const display_list_t *list, const prim_t *p, int y)
{
    const int scale = (p->scale < 1) ? 1 : p->scale;
    const int glyph_row = (y - p->b) / scale;
    if (glyph_row < 0 || glyph_row > 6) {
        return;
    }

    /* Emitting one span per glyph column meant a scale 3 string cost hundreds
     * of three pixel calls, which made text the most expensive thing on most
     * screens. Runs of lit columns are merged into a single span, and glyphs
     * off either edge are rejected before any of that. */
    const int advance = 6 * scale;

    for (int i = 0; i < p->text_len; i++) {
        const int gx = p->a + i * advance;
        if (gx >= W) {
            break;                      /* the rest of the string is off the right */
        }
        if (gx + advance <= 0) {
            continue;                   /* still off the left */
        }

        unsigned char ch = (unsigned char)list->text[p->text_off + i];
        if (ch < 0x20 || ch > 0x7F) {
            ch = '?';
        }
        const uint8_t *glyph = gfx_font5x7[ch - 0x20];

        if (p->bg != GFX_TRANSPARENT) {
            span(gx, gx + advance - 1, p->bg);
        }

        int run = -1;
        for (int col = 0; col <= 5; col++) {
            const bool on = (col < 5) && (((glyph[col] >> glyph_row) & 0x01u) != 0u);
            if (on) {
                if (run < 0) {
                    run = col;
                }
            } else if (run >= 0) {
                span(gx + run * scale, gx + col * scale - 1, p->color);
                run = -1;
            }
        }
    }
}

uint32_t gfx_take_trimmed_rows(void)
{
    const uint32_t n = s_trimmed_rows;
    s_trimmed_rows = 0;
    return n;
}

void gfx_compose_rows(uint16_t *dest, int first_row, int row_count, uint32_t budget_us)
{
    const display_list_t *list = s_active;
    const uint16_t bg = list->bg;
    const bool bytewise = ((bg >> 8) == (bg & 0xFF));

    /* Rows within a buffer cost about the same, so the previous one is a good
     * estimate of the next. Stopping when the estimate says the next row would
     * cross the line keeps the overshoot to a row rather than a buffer, without
     * a clock read per primitive. */
    const int64_t started = esp_timer_get_time();
    int64_t row_end = started;
    int64_t worst_row = 0;
    bool trimming = false;

    for (int r = 0; r < row_count; r++) {
        const int y = first_row + r;
        s_target = dest + (size_t)r * W;

        if (y < 0 || y >= H) {
            memset(s_target, 0, (size_t)W * sizeof(uint16_t));
            continue;
        }

        if (bytewise) {
            memset(s_target, bg & 0xFF, (size_t)W * sizeof(uint16_t));
        } else {
            for (int x = 0; x < W; x++) {
                s_target[x] = bg;
            }
        }

        /* The first row always draws, whatever it costs: something has to come
         * out, and a picture that is only ever background is no better than a
         * frozen one. */
        if (!trimming && r > 0 &&
            (row_end - started) + worst_row > (int64_t)budget_us) {
            trimming = true;
        }
        if (trimming) {
            s_trimmed_rows++;
            continue;
        }

        for (int i = 0; i < list->count; i++) {
            const prim_t *p = &list->prims[i];
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
            case P_RING:
                raster_ring(p, y);
                break;
            case P_TEXT:
                raster_text(list, p, y);
                break;
            case P_ROW_FN:
                p->row_fn(y, s_target, p->row_ctx);
                break;
            }
        }

        const int64_t now = esp_timer_get_time();
        if (now - row_end > worst_row) {
            worst_row = now - row_end;
        }
        row_end = now;
    }
}
