/**
 * Intuitive gravity cube.
 *
 * Port of src/intuitive_cube.c. Accelerometer only: the cube tilts the way the
 * board tilts, with a crosshair marking the gravity vector. A low pass filter
 * keeps it from jittering.
 */

#include <math.h>
#include <stdio.h>

#include "demo_common.h"

#define CUBE_HALF   50.0f
#define CUBE_DIST   200.0f
#define CUBE_FOCAL  120.0f

/* The original used a fixed alpha of 0.1 at roughly 30 Hz, which is a time
 * constant of about 0.3 s. Now that the filter runs at the sensor's 250 Hz, a
 * fixed alpha would make it eight times twitchier, so derive alpha from dt and
 * keep the feel the original had. */
#define FILTER_TAU  0.3f

/* Written on the sensor core, read on the render core. */
static float s_fax, s_fay, s_faz;

static void filter_step(const imu_sample_t *sample)
{
    const float alpha = sample->dt / (FILTER_TAU + sample->dt);

    demo_lock();
    s_fax += alpha * (sample->acc.x - s_fax);
    s_fay += alpha * (sample->acc.y - s_fay);
    s_faz += alpha * (sample->acc.z - s_faz);
    demo_unlock();
}

static void draw_horizon(float tilt_x, float tilt_y)
{
    gfx_line(S(20), DISP_CY, S(220), DISP_CY, GFX_DGREEN);
    gfx_line(DISP_CX, S(20), DISP_CX, S(220), GFX_DGREEN);

    int tx = DISP_CX + (int)(tilt_y * 100.0f * DISP_SCALE);
    int ty = DISP_CY - (int)(tilt_x * 100.0f * DISP_SCALE);

    if (tx < S(20)) {
        tx = S(20);
    }
    if (tx > S(220)) {
        tx = S(220);
    }
    if (ty < S(20)) {
        ty = S(20);
    }
    if (ty > S(220)) {
        ty = S(220);
    }

    gfx_line(tx - S(10), ty, tx + S(10), ty, GFX_YELLOW);
    gfx_line(tx, ty - S(10), tx, ty + S(10), GFX_YELLOW);
    gfx_circle(tx, ty, 6, GFX_YELLOW);
}

static void run(void)
{
    printf("Intuitive gravity cube: the cube follows the board's tilt.\n");

    if (!qmi8658_present()) {
        while (!demo_exit_requested()) {
            demo_show_message("NO IMU", "QMI8658 not detected", GFX_RED);
            demo_delay_ms(200);
        }
        return;
    }

    s_fax = 0.0f;
    s_fay = 0.0f;
    s_faz = 0.0f;
    demo_set_filter(filter_step);

    while (!demo_exit_requested()) {
        float filtered_ax, filtered_ay, filtered_az;
        demo_lock();
        filtered_ax = s_fax;
        filtered_ay = s_fay;
        filtered_az = s_faz;
        demo_unlock();

        /* asinf() is only defined on [-1,1] and a shake can push past 1 g. */
        float clamped_x = filtered_ax;
        float clamped_y = filtered_ay;
        if (clamped_x > 1.0f) {
            clamped_x = 1.0f;
        }
        if (clamped_x < -1.0f) {
            clamped_x = -1.0f;
        }
        if (clamped_y > 1.0f) {
            clamped_y = 1.0f;
        }
        if (clamped_y < -1.0f) {
            clamped_y = -1.0f;
        }

        float tilt_x = asinf(clamped_x);
        float tilt_y = asinf(clamped_y);

        if (tilt_x > PI / 4) {
            tilt_x = PI / 4;
        }
        if (tilt_x < -PI / 4) {
            tilt_x = -PI / 4;
        }
        if (tilt_y > PI / 4) {
            tilt_y = PI / 4;
        }
        if (tilt_y < -PI / 4) {
            tilt_y = -PI / 4;
        }

        demo_frame_begin(GFX_BLACK);

        vertex_t rotated[8];
        for (int i = 0; i < 8; i++) {
            rotated[i] = demo_cube_vertices[i];
            rotate_x(&rotated[i], tilt_x);
            rotate_y(&rotated[i], tilt_y);
        }

        draw_horizon(filtered_ax, filtered_ay);
        demo_draw_cube(rotated, CUBE_DIST, CUBE_FOCAL);

        char line[48];
        snprintf(line, sizeof(line), "tilt X %+5.1f" GFX_DEG "  Y %+5.1f" GFX_DEG,
                 (double)(tilt_x * RAD_TO_DEG), (double)(tilt_y * RAD_TO_DEG));
        gfx_text_centered(DISP_CX, DEMO_ROW_TOP(0), line, GFX_WHITE, 2);

        snprintf(line, sizeof(line), "acc %+.2f %+.2f %+.2f",
                 (double)filtered_ax, (double)filtered_ay, (double)filtered_az);
        gfx_text_centered(DISP_CX, DEMO_ROW_BOTTOM(0), line, GFX_GREY, 2);
        demo_draw_exit_hint();

        demo_frame_end();

        printf("\rTilt X: %6.2f Y: %6.2f | Acc: %.2f %.2f %.2f      ",
               (double)(tilt_x * RAD_TO_DEG), (double)(tilt_y * RAD_TO_DEG),
               (double)filtered_ax, (double)filtered_ay, (double)filtered_az);

    }
    demo_set_filter(NULL);
    printf("\n");
}

const demo_t demo_intuitive_cube = {
    .name = "intuitive_cube",
    .summary = "accelerometer tilt cube",
    .run = run,
};
