/**
 * Touch panel test.
 *
 * New for this board: the RP2350-LCD-1.28 had no touch screen. Draw with a
 * finger, watch the reported coordinates and gestures, and use the swatches
 * along the bottom to change colour. Tap CLEAR to wipe the canvas.
 */

#include <stdio.h>

#include "demo_common.h"

#define SWATCH_COUNT 6
#define SWATCH_Y     (DISP_H - 130)
#define SWATCH_SIZE  44

static const uint16_t s_palette[SWATCH_COUNT] = {
    GFX_WHITE, GFX_RED, GFX_GREEN, GFX_CYAN, GFX_YELLOW, GFX_MAGENTA,
};

#define TRAIL_MAX 512

typedef struct {
    int16_t x, y;
    uint16_t color;
    bool start;         /* first sample of a stroke, so no line back */
} trail_point_t;

static trail_point_t s_trail[TRAIL_MAX];
static int s_trail_count;

static int swatch_x(int index)
{
    const int total = SWATCH_COUNT * SWATCH_SIZE + (SWATCH_COUNT - 1) * 8;
    return DISP_CX - total / 2 + index * (SWATCH_SIZE + 8);
}

static void draw_canvas(void)
{
    for (int i = 0; i < s_trail_count; i++) {
        if (s_trail[i].start || i == 0) {
            gfx_fill_circle(s_trail[i].x, s_trail[i].y, 3, s_trail[i].color);
        } else {
            gfx_thick_line(s_trail[i - 1].x, s_trail[i - 1].y,
                           s_trail[i].x, s_trail[i].y, 6, s_trail[i].color);
        }
    }
}

static void run(void)
{
    printf("Touch test: drag to draw, tap a swatch to change colour.\n");

    if (!cst820_present()) {
        while (!demo_exit_requested()) {
            demo_show_message("NO TOUCH", "CST820 not detected", GFX_RED);
            demo_delay_ms(200);
        }
        return;
    }

    int color_index = 0;
    bool was_pressed = false;
    touch_gesture_t last_gesture = TOUCH_GESTURE_NONE;
    uint32_t samples = 0;

    while (!demo_exit_requested()) {
        touch_state_t touch;
        const bool pressed = demo_touch(&touch);

        if (pressed) {
            samples++;
            if (touch.gesture != TOUCH_GESTURE_NONE) {
                last_gesture = touch.gesture;
            }

            if (touch.y > SWATCH_Y && touch.y < SWATCH_Y + SWATCH_SIZE) {
                for (int i = 0; i < SWATCH_COUNT; i++) {
                    const int x = swatch_x(i);
                    if (touch.x >= x && touch.x < x + SWATCH_SIZE) {
                        color_index = i;
                    }
                }
            } else if (touch.y > SWATCH_Y + SWATCH_SIZE + 6 && !was_pressed) {
                s_trail_count = 0;      /* CLEAR strip */
            } else if (s_trail_count < TRAIL_MAX) {
                s_trail[s_trail_count++] = (trail_point_t){
                    .x = (int16_t)touch.x,
                    .y = (int16_t)touch.y,
                    .color = s_palette[color_index],
                    .start = !was_pressed,
                };
            }

            printf("\rtouch x=%3u y=%3u gesture=%-13s points=%lu   ",
                   touch.x, touch.y, cst820_gesture_name(touch.gesture),
                   (unsigned long)samples);
        }
        was_pressed = pressed;

        demo_frame_begin(GFX_BLACK);

        gfx_circle(DISP_CX, DISP_CY, 239, GFX_DGREY);
        draw_canvas();

        if (pressed) {
            gfx_circle(touch.x, touch.y, 22, s_palette[color_index]);
            gfx_printf(S(10), S(10), GFX_WHITE, 2, "x %3u  y %3u", touch.x, touch.y);
        } else {
            gfx_text(S(10), S(10), "no contact", GFX_DGREY, 2);
        }
        gfx_printf(S(10), S(22), GFX_GREY, 2, "gesture %s",
                   cst820_gesture_name(last_gesture));

        for (int i = 0; i < SWATCH_COUNT; i++) {
            const int x = swatch_x(i);
            gfx_fill_rect(x, SWATCH_Y, SWATCH_SIZE, SWATCH_SIZE, s_palette[i]);
            if (i == color_index) {
                gfx_rect(x - 3, SWATCH_Y - 3, SWATCH_SIZE + 6, SWATCH_SIZE + 6, GFX_WHITE);
            }
        }
        gfx_text_centered(DISP_CX, SWATCH_Y + SWATCH_SIZE + 14, "TAP BELOW TO CLEAR",
                          GFX_DGREY, 2);
        demo_draw_exit_hint();

        demo_frame_end();
        demo_delay_ms(10);
    }
    printf("\n");
    s_trail_count = 0;
}

const demo_t demo_touch_test = {
    .name = "touch_test",
    .summary = "CST820 touch panel and gestures",
    .run = run,
};
