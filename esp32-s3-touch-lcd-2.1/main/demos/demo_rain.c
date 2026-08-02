/**
 * Rain on a puddle.
 *
 * A port of murr2k/bbb-neopixel-rain, which runs this on an 8x8 WS2812B matrix
 * off a BeagleBone. Top down view: drops land and expand as glowing blue rings.
 *
 * The original, per pixel:
 *
 *     v = sum over drops of exp(-(dist - radius)^2 / 2 sigma^2) * (1 - t/max_age)
 *     rgb = (30 v^3, 140 v^2, 255 v)
 *
 * Two details of that matter more than they look.
 *
 * The sum is over *intensity*, and the colour curve is applied afterwards. Where
 * two rings cross, v adds and then goes through a curve where red rises as the
 * cube and green as the square, so the crossing flares to cyan white while
 * either ring alone is deep blue. Summing colours instead, which is the obvious
 * way to do it with a painter's algorithm, loses that completely: it just gets a
 * bit bluer. So intensity is accumulated per pixel across a row and mapped
 * through a lookup table at the end, which also happens to be cheaper, a byte
 * add per pixel instead of a read, unpack, saturate and repack.
 *
 * The profile is also not symmetric here. A spreading ripple has a sharp front
 * and a trailing wake, so the Gaussian is tighter outside the crest than inside.
 * The original's is symmetric; on an 8x8 panel, where a ring spans the whole
 * display, the difference is invisible. On 480 pixels it is the difference
 * between a hoop and a ripple.
 *
 * Composition runs in the panel's interrupt, which has no floating point and a
 * fixed deadline, so the field is reduced once per frame in the task to a set of
 * annuli with byte intensities. The interrupt only fills spans and looks up
 * colours.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "demo_common.h"

/* The original's parameters, scaled from an 8 pixel panel at 30 fps. */
#define RAIN_SPAWN_PER_SEC  2.1f    /* was spawn_prob 0.07 per frame at 30 fps */
#define RAIN_RING_SPEED     500.0f  /* was ring_speed 0.30 px/frame */
#define RAIN_RING_SIGMA     26.0f   /* was ring_sigma 0.55 px */
#define RAIN_MAX_AGE        0.733f  /* was max_age 22 frames */
#define RAIN_MAX_ACTIVE     3       /* as the original */

/* How much longer the wake is than the front. 1.0 would be the original's
 * symmetric ring. */
#define RAIN_TRAIL          2.6f
#define RAIN_LEAD_SPAN      2.0f    /* sigma outside the crest */
#define RAIN_TRAIL_SPAN     2.5f    /* sigma inside, before the trail factor */

#define RAIN_BANDS          9

/* Blended pixels per refresh across all drops. The interrupt has a fixed time
 * per row and the cost of a ring is about 2 pi r times its width, so the width
 * is capped to hold that product roughly constant. A ripple that thins as it
 * spreads is what water does anyway. */
#define RAIN_AREA_BUDGET    240000.0f

/* Bands are contiguous, so the outer edge of one is the inner edge of the next.
 * Storing the edges rather than each band's pair of radii halves the square
 * roots the interrupt has to take, and those turned out to dominate. */
typedef struct {
    int16_t cx, cy;
    int16_t edge[RAIN_BANDS + 1];   /* ascending radii */
    uint8_t level[RAIN_BANDS];      /* intensity between edge[k] and edge[k+1] */
    uint8_t bands;
} ring_t;

typedef struct {
    ring_t rings[RAIN_MAX_ACTIVE];
    uint8_t count;
} rain_field_t;

typedef struct {
    float cx, cy;
    float age;
    bool alive;
} drop_t;

static drop_t s_drops[RAIN_MAX_ACTIVE];

/* Two fields, alternating with the display list: the interrupt reads one while
 * the next frame is built in the other. */
static rain_field_t s_fields[2];
static uint16_t s_water[256];
static uint8_t s_acc[DISP_W];
static float s_quality = 1.0f;

/* Trim when composition approaches its deadline, recover slowly below it. */
#define RAIN_TARGET_US  ((LCD_2IN1_ComposeBudgetUs() * 85u) / 100u)
#define RAIN_RELAX_US   ((LCD_2IN1_ComposeBudgetUs() * 55u) / 100u)

static float frand(void)
{
    return (float)rand() / (float)RAND_MAX;
}

/** The original's water_rgb(), tabulated. Blue rises linearly, green as the
 *  square and red as the cube, so a faint ring is deep blue and anything that
 *  reaches full intensity, which in practice means two rings crossing, goes
 *  cyan white. */
static void build_water_table(void)
{
    for (int i = 0; i < 256; i++) {
        const float v = (float)i / 255.0f;
        s_water[i] = GFX_RGB((int)(30.0f * v * v * v),
                             (int)(140.0f * v * v),
                             (int)(255.0f * v));
    }
}

static void adapt_quality(void)
{
    const uint32_t used = LCD_2IN1_ComposeMaxUs();      /* reading resets it */
    if (used == 0) {
        return;
    }
    if (used > RAIN_TARGET_US) {
        s_quality -= 0.05f;
    } else if (used < RAIN_RELAX_US) {
        s_quality += 0.004f;
    }
    if (s_quality < 0.3f) {
        s_quality = 0.3f;
    }
    if (s_quality > 1.0f) {
        s_quality = 1.0f;
    }
}

static void spawn(drop_t *d)
{
    /* Centres may be off panel, as in the original, so arcs sweep in from the
     * bezel instead of every ripple starting whole. */
    d->cx = -0.15f * DISP_W + frand() * 1.3f * DISP_W;
    d->cy = -0.15f * DISP_H + frand() * 1.3f * DISP_H;
    d->age = 0.0f;
    d->alive = true;
}

static void step_drops(float dt)
{
    int active = 0;
    for (int i = 0; i < RAIN_MAX_ACTIVE; i++) {
        if (!s_drops[i].alive) {
            continue;
        }
        s_drops[i].age += dt;
        if (s_drops[i].age >= RAIN_MAX_AGE) {
            s_drops[i].alive = false;
        } else {
            active++;
        }
    }

    if (active < RAIN_MAX_ACTIVE && frand() < RAIN_SPAWN_PER_SEC * dt) {
        for (int i = 0; i < RAIN_MAX_ACTIVE; i++) {
            if (!s_drops[i].alive) {
                spawn(&s_drops[i]);
                break;
            }
        }
    }
}

/** Reduce one drop to annuli with byte intensities. Runs in the task, so the
 *  Gaussian is evaluated in floating point here and never in the interrupt. */
static void build_ring(const drop_t *d, int active, ring_t *out)
{
    const float radius = d->age * RAIN_RING_SPEED;
    const float fade = 1.0f - d->age / RAIN_MAX_AGE;

    float sigma_out = RAIN_RING_SIGMA * s_quality;
    float sigma_in = sigma_out * RAIN_TRAIL;

    /* Hold the blended area roughly constant as the ring grows. */
    if (radius > 1.0f) {
        const float width = RAIN_LEAD_SPAN * sigma_out + RAIN_TRAIL_SPAN * sigma_in;
        const float affordable =
            RAIN_AREA_BUDGET / ((float)(active < 1 ? 1 : active) * 6.2832f * radius);
        if (width > affordable) {
            const float shrink = affordable / width;
            sigma_out *= shrink;
            sigma_in *= shrink;
        }
    }
    if (sigma_out < 1.5f) {
        sigma_out = 1.5f;
        sigma_in = sigma_out * RAIN_TRAIL;
    }

    const float inner_edge = radius - RAIN_TRAIL_SPAN * sigma_in;
    const float outer_edge = radius + RAIN_LEAD_SPAN * sigma_out;
    const float step = (outer_edge - inner_edge) / (float)RAIN_BANDS;

    out->cx = (int16_t)d->cx;
    out->cy = (int16_t)d->cy;
    out->bands = 0;

    for (int k = 0; k < RAIN_BANDS; k++) {
        const float lo = inner_edge + (float)k * step;
        const float hi = lo + step;
        if (hi <= 0.0f) {
            continue;
        }

        /* Distance of this band's middle from the crest, with the wake side
         * spread wider than the front. */
        const float offset = lo + 0.5f * step - radius;
        const float sigma = (offset >= 0.0f) ? sigma_out : sigma_in;
        const float v = expf(-(offset * offset) / (2.0f * sigma * sigma)) * fade;

        int level = (int)(v * 255.0f);
        if (level > 255) {
            level = 255;
        }
        if (level <= 2) {
            continue;                   /* nothing this dim survives the table */
        }

        const int slot = out->bands;
        if (slot == 0) {
            out->edge[0] = (int16_t)((lo < 0.0f) ? 0.0f : lo);
        }
        out->edge[slot + 1] = (int16_t)hi;
        out->level[slot] = (uint8_t)level;
        out->bands++;
    }
}

/* --- the interrupt side ---------------------------------------------------- */

static inline uint32_t isqrt_u32(uint32_t v)
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

/* Widest extent touched on this row, so the colour pass only has to convert
 * what was actually drawn instead of all 480 columns. */
static int s_touch_lo;
static int s_touch_hi;

static inline void add_span(int x0, int x1, uint8_t level)
{
    if (x0 < 0) {
        x0 = 0;
    }
    if (x1 > DISP_W - 1) {
        x1 = DISP_W - 1;
    }
    if (x0 > x1) {
        return;
    }
    if (x0 < s_touch_lo) {
        s_touch_lo = x0;
    }
    if (x1 > s_touch_hi) {
        s_touch_hi = x1;
    }

    for (int x = x0; x <= x1; x++) {
        const int sum = (int)s_acc[x] + (int)level;
        s_acc[x] = (uint8_t)((sum > 255) ? 255 : sum);
    }
}

/** Composed straight into the row, in the panel's interrupt: accumulate every
 *  ring's intensity, then map once. Summing here rather than in colour is what
 *  makes two rings crossing flare white. */
static void rain_row(int y, uint16_t *row, void *ctx)
{
    const rain_field_t *field = (const rain_field_t *)ctx;

    /* Cleared in full: clearing only what this row touched would leave stale
     * counts where a later row reaches wider. 480 bytes is not worth the risk. */
    memset(s_acc, 0, sizeof(s_acc));
    s_touch_lo = DISP_W;
    s_touch_hi = -1;

    for (int i = 0; i < field->count; i++) {
        const ring_t *ring = &field->rings[i];
        const int dy = y - ring->cy;
        const int dy2 = dy * dy;

        /* One square root per edge, shared by the bands either side of it. */
        int extent[RAIN_BANDS + 1];
        for (int k = 0; k <= ring->bands; k++) {
            const int e2 = (int)ring->edge[k] * ring->edge[k] - dy2;
            extent[k] = (e2 > 0) ? (int)isqrt_u32((uint32_t)e2) : -1;
        }

        for (int k = 0; k < ring->bands; k++) {
            const int outer = extent[k + 1];
            if (outer < 0) {
                continue;               /* this row misses the band entirely */
            }
            const int inner = extent[k];
            if (inner < 0) {
                add_span(ring->cx - outer, ring->cx + outer, ring->level[k]);
            } else {
                add_span(ring->cx - outer, ring->cx - inner, ring->level[k]);
                add_span(ring->cx + inner, ring->cx + outer, ring->level[k]);
            }
        }
    }

    if (s_touch_hi < s_touch_lo) {
        memset(row, 0, (size_t)DISP_W * sizeof(uint16_t));
        return;
    }

    /* Only the touched span needs converting; the rest is background. */
    if (s_touch_lo > 0) {
        memset(row, 0, (size_t)s_touch_lo * sizeof(uint16_t));
    }
    if (s_touch_hi < DISP_W - 1) {
        memset(&row[s_touch_hi + 1], 0,
               (size_t)(DISP_W - 1 - s_touch_hi) * sizeof(uint16_t));
    }
    for (int x = s_touch_lo; x <= s_touch_hi; x++) {
        row[x] = s_water[s_acc[x]];
    }
}

static void run(void)
{
    printf("Rain on a puddle. A port of bbb-neopixel-rain, 8x8 matrix to 480x480.\n");
    printf("Rings sum in intensity, so crossings flare. Touch or press a key to leave.\n");

    build_water_table();
    srand((unsigned)demo_micros());
    for (int i = 0; i < RAIN_MAX_ACTIVE; i++) {
        s_drops[i].alive = false;
    }
    spawn(&s_drops[0]);

    uint64_t last_us = demo_micros();
    uint32_t frames = 0;
    s_quality = 1.0f;

    for (;;) {
        /* A screensaver goes away on any contact, not just the exit gesture. */
        if (demo_exit_requested() || demo_read_char() >= 0) {
            break;
        }
        touch_state_t touch;
        if (demo_touch(&touch)) {
            break;
        }

        const uint64_t now = demo_micros();
        float dt = (float)(now - last_us) / 1000000.0f;
        last_us = now;
        if (dt > 0.1f) {
            dt = 0.1f;
        }

        adapt_quality();
        step_drops(dt);

        int active = 0;
        for (int i = 0; i < RAIN_MAX_ACTIVE; i++) {
            if (s_drops[i].alive) {
                active++;
            }
        }

        rain_field_t *field = &s_fields[frames & 1u];
        field->count = 0;
        for (int i = 0; i < RAIN_MAX_ACTIVE; i++) {
            if (s_drops[i].alive) {
                build_ring(&s_drops[i], active, &field->rings[field->count]);
                field->count++;
            }
        }

        demo_frame_begin(GFX_BLACK);
        gfx_row_painter(rain_row, field);
        demo_frame_end();

        if (++frames % 240 == 0) {
            printf("rain: quality %.2f, %d active\n", (double)s_quality, active);
        }
    }
    printf("\n");
}

const demo_t demo_rain = {
    .name = "rain",
    .summary = "rain on a puddle, from bbb-neopixel-rain",
    .run = run,
};
