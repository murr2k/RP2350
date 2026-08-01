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
#define FILTER_ALPHA 0.1f

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

    float filtered_ax = 0.0f;
    float filtered_ay = 0.0f;
    float filtered_az = 0.0f;

    while (!demo_exit_requested()) {
        vector3f_t acc;
        if (!demo_read_imu(&acc, NULL)) {
            demo_delay_ms(10);
            continue;
        }

        filtered_ax = FILTER_ALPHA * acc.x + (1.0f - FILTER_ALPHA) * filtered_ax;
        filtered_ay = FILTER_ALPHA * acc.y + (1.0f - FILTER_ALPHA) * filtered_ay;
        filtered_az = FILTER_ALPHA * acc.z + (1.0f - FILTER_ALPHA) * filtered_az;

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

        gfx_printf(S(12), S(12), GFX_WHITE, 2, "tilt X %6.1f" GFX_DEG,
                   (double)(tilt_x * RAD_TO_DEG));
        gfx_printf(S(12), S(24), GFX_WHITE, 2, "tilt Y %6.1f" GFX_DEG,
                   (double)(tilt_y * RAD_TO_DEG));
        gfx_printf(S(12), S(200), GFX_GREY, 2, "acc %+.2f %+.2f %+.2f",
                   (double)filtered_ax, (double)filtered_ay, (double)filtered_az);
        demo_draw_exit_hint();

        demo_frame_end();

        printf("\rTilt X: %6.2f Y: %6.2f | Acc: %.2f %.2f %.2f      ",
               (double)(tilt_x * RAD_TO_DEG), (double)(tilt_y * RAD_TO_DEG),
               (double)filtered_ax, (double)filtered_ay, (double)filtered_az);

        demo_delay_ms(33);
    }
    printf("\n");
}

const demo_t demo_intuitive_cube = {
    .name = "intuitive_cube",
    .summary = "accelerometer tilt cube",
    .run = run,
};
