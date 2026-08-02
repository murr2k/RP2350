/**
 * Display bring-up test.
 *
 * Work-alike of the RP2350 minimal_lcd / waveshare_test / spi_test images: the
 * first thing to run on a new board to prove the panel, the colour order and
 * the backlight are all sane. Nothing here touches the IMU.
 */

#include <math.h>
#include <stdio.h>

#include "demo_common.h"

static void solid(uint16_t color, const char *label, uint16_t text_color)
{
    demo_frame_begin(color);
    gfx_text_centered(DISP_CX, DISP_CY - 8, label, text_color, 3);
    demo_frame_end();
    demo_delay_ms(700);
}

/* Nothing describes a per pixel gradient as a primitive, so it paints its own
 * rows. Recorded like anything else, so the label below still lands on top.
 *
 * This runs in the panel's interrupt for every visible row, so the arithmetic
 * that depends only on x is done once into tables and the inner loop is two
 * lookups. Doing it per pixel made this the most expensive thing in the whole
 * firmware. */
static uint8_t s_grad_r[DISP_W];
static uint8_t s_grad_b[DISP_W];
static bool s_grad_ready;

static void gradient_tables(void)
{
    for (int x = 0; x < DISP_W; x++) {
        s_grad_r[x] = (uint8_t)(x * 255 / DISP_W);
        s_grad_b[x] = (uint8_t)((x * 255) / (DISP_W + DISP_H));
    }
    s_grad_ready = true;
}

static void gradient_row(int y, uint16_t *row, void *ctx)
{
    (void)ctx;
    const int g = y * 255 / DISP_H;
    const int b_row = 255 - (y * 255) / (DISP_W + DISP_H);

    for (int x = 0; x < DISP_W; x++) {
        int b = b_row - (int)s_grad_b[x];
        if (b < 0) {
            b = 0;
        }
        row[x] = GFX_RGB(s_grad_r[x], g, b);
    }
}

static void gradient(void)
{
    if (!s_grad_ready) {
        gradient_tables();
    }
    demo_frame_begin(GFX_BLACK);
    gfx_row_painter(gradient_row, NULL);
    gfx_text_centered(DISP_CX, 40, "RGB565 GRADIENT", GFX_WHITE, 2);
    demo_frame_end();
    demo_delay_ms(1500);
}

static void geometry(void)
{
    demo_frame_begin(GFX_BLACK);

    /* Concentric rings show whether the round panel is centred. */
    for (int r = 40; r < 240; r += 40) {
        gfx_circle(DISP_CX, DISP_CY, r, GFX_DGREY);
    }
    gfx_circle(DISP_CX, DISP_CY, 239, GFX_CYAN);

    gfx_line(0, DISP_CY, DISP_W - 1, DISP_CY, GFX_DGREEN);
    gfx_line(DISP_CX, 0, DISP_CX, DISP_H - 1, GFX_DGREEN);

    gfx_fill_rect(DISP_CX - 60, DISP_CY - 60, 120, 120, GFX_RED);
    gfx_rect(DISP_CX - 80, DISP_CY - 80, 160, 160, GFX_YELLOW);
    gfx_fill_circle(DISP_CX, DISP_CY, 30, GFX_BLUE);

    gfx_text_centered(DISP_CX, 60, "GEOMETRY", GFX_WHITE, 2);
    gfx_text_centered(DISP_CX, DISP_H - 90, "480x480 ST7701S", GFX_WHITE, 2);
    demo_frame_end();
    demo_delay_ms(1500);
}

static void text_sizes(void)
{
    demo_frame_begin(GFX_BLACK);
    gfx_text_centered(DISP_CX, 110, "scale 1 abcdefghij 0123456789", GFX_WHITE, 1);
    gfx_text_centered(DISP_CX, 150, "scale 2 abcdef 012345", GFX_GREEN, 2);
    gfx_text_centered(DISP_CX, 200, "scale 3 abc 0123", GFX_YELLOW, 3);
    gfx_text_centered(DISP_CX, 260, "scale 4 AB 45", GFX_CYAN, 4);
    gfx_text_centered(DISP_CX, 330, "90" GFX_DEG "C  -12.3" GFX_DEG, GFX_MAGENTA, 3);
    demo_frame_end();
    demo_delay_ms(2000);
}

static void backlight_sweep(void)
{
    demo_frame_begin(GFX_BLACK);
    gfx_text_centered(DISP_CX, DISP_CY - 40, "BACKLIGHT SWEEP", GFX_WHITE, 2);
    gfx_fill_rect(DISP_CX - 150, DISP_CY, 300, 40, GFX_WHITE);
    demo_frame_end();

    for (int level = 100; level >= 10 && !demo_exit_requested(); level -= 10) {
        DEV_SET_PWM((uint8_t)level);
        demo_delay_ms(120);
    }
    for (int level = 10; level <= 100 && !demo_exit_requested(); level += 10) {
        DEV_SET_PWM((uint8_t)level);
        demo_delay_ms(120);
    }
    DEV_SET_PWM(100);
}

/* Angle comes in as an accumulated time, not a frame count. Stepping per frame
 * meant the balls sped up threefold when the sleeps were removed, which the
 * panel showed as smearing rather than six discrete dots. */
static void animated_page(float t, uint32_t frame)
{
    demo_frame_begin(GFX_BLACK);

    for (int i = 0; i < 6; i++) {
        const float a = t + (float)i * (TWO_PI / 6.0f);
        const int x = DISP_CX + (int)(150.0f * cosf(a));
        const int y = DISP_CY + (int)(150.0f * sinf(a));
        const uint16_t colors[6] = {GFX_RED,  GFX_GREEN,   GFX_BLUE,
                                    GFX_YELLOW, GFX_MAGENTA, GFX_CYAN};
        gfx_fill_circle(x, y, 26, colors[i]);
    }

    gfx_circle(DISP_CX, DISP_CY, 239, GFX_DGREY);
    gfx_text_centered(DISP_CX, DISP_CY - 30, "DISPLAY TEST", GFX_WHITE, 3);
    gfx_printf(DISP_CX - 60, DISP_CY + 10, GFX_GREY, 2, "frame %lu",
               (unsigned long)frame);
    demo_draw_exit_hint();

    demo_frame_end();
}

static void run(void)
{
    printf("Display test: colour fields, gradient, geometry, text, backlight.\n");

    solid(GFX_RED, "RED", GFX_WHITE);
    if (demo_exit_requested()) {
        return;
    }
    solid(GFX_GREEN, "GREEN", GFX_BLACK);
    solid(GFX_BLUE, "BLUE", GFX_WHITE);
    solid(GFX_WHITE, "WHITE", GFX_BLACK);

    if (!demo_exit_requested()) {
        gradient();
    }
    if (!demo_exit_requested()) {
        geometry();
    }
    if (!demo_exit_requested()) {
        text_sizes();
    }
    if (!demo_exit_requested()) {
        backlight_sweep();
    }

    uint32_t frame = 0;
    float angle = 0.0f;
    uint64_t last_us = demo_micros();

    while (!demo_exit_requested()) {
        angle += 1.6f * demo_delta_seconds(&last_us);   /* was 0.08 per frame */
        animated_page(angle, frame++);
    }
}

const demo_t demo_display_test = {
    .name = "display_test",
    .summary = "panel, colour and backlight check",
    .run = run,
};
