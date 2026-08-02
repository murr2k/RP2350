/**
 * Rain on a puddle.
 *
 * A port of murr2k/bbb-neopixel-rain, which runs this on an 8x8 WS2812B matrix
 * off a BeagleBone. Top down view: drops land and expand as glowing blue rings.
 * The model is the original's, scaled from an 8 pixel panel at 30 fps to a 480
 * pixel one at the panel's own rate.
 *
 * Original, per pixel:
 *
 *     v = sum over drops of exp(-(dist - radius)^2 / 2 sigma^2) * (1 - t/max_age)
 *     rgb = (30 v^3, 140 v^2, 255 v)
 *
 * Evaluating that per pixel is out of the question here: composition happens in
 * the panel's interrupt, which has no floating point and about 68 us per row.
 * But the field is radially symmetric, so the same shape comes from a stack of
 * annuli whose colours follow the Gaussian. Those are computed once per frame in
 * the task, where floats are allowed, and the interrupt only has to fill spans.
 *
 * They add rather than overwrite, so two ripples crossing brighten each other
 * the way the summation in the original does.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "demo_common.h"

/* The original's parameters, scaled. An 8 pixel panel became 480, and 30 fps
 * became whatever the panel runs at, so the rates are per second here. */
#define RAIN_SPAWN_PER_SEC  2.1f    /* was spawn_prob 0.07 per frame at 30 fps */
#define RAIN_RING_SPEED     420.0f  /* was ring_speed 0.30 px/frame, 540 to scale */
#define RAIN_RING_SIGMA     18.0f   /* was ring_sigma 0.55 px, 33 to scale */
#define RAIN_MAX_AGE        0.733f  /* was max_age 22 frames */

/* Ring speed and sigma are below their scaled values on purpose. The cost of
 * this effect is the blended area, roughly 2 pi r sigma per band, and a ring is
 * recomposed on every panel refresh whether or not the animation advanced. At
 * the scaled figures that came to 499 us against a 342 us deadline. A tighter,
 * slightly slower ripple fits, at the price of a sharper ring than the 8x8
 * original's. */
/* The original allowed three at once on a panel where a ring was eight pixels
 * across. Here a ring sweeps 400 pixels and each one is composed afresh on
 * every refresh, whether or not anything moved, so three of them do not fit in
 * the 342 us the interrupt has per bounce buffer. Two do. */
#define RAIN_MAX_ACTIVE     2

/* Steps across the Gaussian. More steps is a smoother ring, and more work per
 * row in the interrupt: each band costs two spans of additive blending on every
 * row it crosses. Thirteen bands measured 455 us against a 342 us budget and
 * stalled the composer, so seven it is, spread over a slightly tighter span
 * where most of the Gaussian's energy lives anyway. */
#define RAIN_BANDS          5
#define RAIN_SIGMA_SPAN     1.75f   /* sigma either side of the crest */

typedef struct {
    float cx, cy;
    float age;
    bool alive;
} drop_t;

static drop_t s_drops[RAIN_MAX_ACTIVE];

/* Composition happens in the panel's interrupt with a hard 342 us per bounce
 * buffer, and the cost of this effect is its blended area, which is roughly
 * 2 pi r sigma per drop. Three attempts to pick parameters that fit by hand all
 * missed, in both directions, so the demo measures instead: it watches how long
 * composition actually took and trims itself until it fits, then creeps back up
 * while there is headroom.
 *
 * Overrunning is not a graceful degradation. The composer falls behind the DMA,
 * the bounce position stops wrapping, the frame boundary event never arrives,
 * and every frame waits out the 100 ms backstop. So the target leaves room. */
#define RAIN_BUDGET_US      342
#define RAIN_TARGET_US      290
#define RAIN_RELAX_US       190

/* Blended pixels per refresh, across all drops, that comfortably fit the
 * deadline. Used feed forward: cost is about 2 pi r per unit of ring width, so
 * the width is capped to hold the product roughly constant. A ripple that thins
 * as it spreads is also what water does, so the cheap answer is the pretty
 * one. The feedback below then only has to absorb the error in this estimate. */
#define RAIN_AREA_BUDGET    110000.0f

static float s_quality = 1.0f;

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
    if (s_quality < 0.25f) {
        s_quality = 0.25f;
    }
    if (s_quality > 1.0f) {
        s_quality = 1.0f;
    }
}

static float frand(void)
{
    return (float)rand() / (float)RAND_MAX;
}

/** The original's water_rgb(): blue rises linearly, green quadratically, red
 *  cubically, so a faint ring is deep blue and the crest goes cyan white. */
static uint16_t water_color(float v)
{
    if (v <= 0.0f) {
        return 0;
    }
    if (v > 1.0f) {
        v = 1.0f;
    }
    const int r = (int)(30.0f * v * v * v);
    const int g = (int)(140.0f * v * v);
    const int b = (int)(255.0f * v);
    return GFX_RGB(r, g, b);
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
    /* The first thing to give up under pressure is a second simultaneous
     * ripple, which costs twice as much as anything else on the list. */
    const int cap = (s_quality > 0.6f) ? RAIN_MAX_ACTIVE : 1;
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

    if (active < cap && frand() < RAIN_SPAWN_PER_SEC * dt) {
        for (int i = 0; i < RAIN_MAX_ACTIVE; i++) {
            if (!s_drops[i].alive) {
                spawn(&s_drops[i]);
                break;
            }
        }
    }
}

static void draw_drop(const drop_t *d, int active)
{
    const float radius = d->age * RAIN_RING_SPEED;
    const float fade = 1.0f - d->age / RAIN_MAX_AGE;
    const float two_sigma_sq = 2.0f * RAIN_RING_SIGMA * RAIN_RING_SIGMA;

    /* Band count does not change the pixel total, only how finely the same
     * width is sliced. Width does, so that is what gets capped. */
    float reach = RAIN_SIGMA_SPAN * RAIN_RING_SIGMA * s_quality;
    if (radius > 1.0f) {
        const float affordable =
            RAIN_AREA_BUDGET / ((float)(active < 1 ? 1 : active) * 6.2832f * radius);
        if (2.0f * reach > affordable) {
            reach = 0.5f * affordable;
        }
    }
    if (reach < 3.0f) {
        reach = 3.0f;
    }

    const float span = 2.0f * reach;
    const float step = span / (float)RAIN_BANDS;

    const int cx = (int)d->cx;
    const int cy = (int)d->cy;

    for (int k = 0; k < RAIN_BANDS; k++) {
        const float inner = radius - reach + (float)k * step;
        const float outer = inner + step;
        if (outer <= 0.0f) {
            continue;
        }

        /* Intensity at the middle of the band, exactly the original's term. */
        const float centre = inner + 0.5f * step - radius;
        const float v = expf(-(centre * centre) / two_sigma_sq) * fade;
        const uint16_t color = water_color(v);
        if (color == 0) {
            continue;
        }

        /* Nothing to draw once the band has swept past the far corner. */
        if (inner > 1.6f * DISP_W) {
            continue;
        }
        gfx_ring_add(cx, cy, (int)inner, (int)outer, color);
    }
}

static void run(void)
{
    printf("Rain on a puddle. A port of bbb-neopixel-rain, 8x8 matrix to 480x480.\n");
    printf("Touch the screen or press any key to leave.\n");

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

        demo_frame_begin(GFX_BLACK);
        for (int i = 0; i < RAIN_MAX_ACTIVE; i++) {
            if (s_drops[i].alive) {
                draw_drop(&s_drops[i], active);
            }
        }
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
