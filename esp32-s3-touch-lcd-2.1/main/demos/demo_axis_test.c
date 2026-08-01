/**
 * IMU axis test.
 *
 * Port of src/axis_test.c. Draws the raw accelerometer and gyroscope vectors
 * from the centre of the screen so you can see, at a glance, which way each
 * axis points. Hold the board flat and Z should read about 1 g.
 */

#include <math.h>
#include <stdio.h>

#include "demo_common.h"

static void draw_vector(float x, float y, float z, uint16_t color)
{
    const int endx = DISP_CX + (int)(x * S(50));
    const int endy = DISP_CY - (int)(y * S(50));   /* screen Y grows downward */

    int thickness = 2;
    if (z > 0.1f) {
        thickness = 4;
    }
    if (z > 0.5f) {
        thickness = 6;
    }
    gfx_thick_line(DISP_CX, DISP_CY, endx, endy, thickness, color);
}

static void draw_reference(void)
{
    gfx_line(S(20), DISP_CY, S(220), DISP_CY, GFX_DGREEN);
    gfx_line(DISP_CX, S(20), DISP_CX, S(220), GFX_DGREEN);

    gfx_text(S(222), DISP_CY - 6, "+X", GFX_DGREEN, 2);
    gfx_text(DISP_CX + 6, S(16), "+Y", GFX_DGREEN, 2);
    gfx_circle(DISP_CX, DISP_CY, S(50), GFX_DGREY);
}

static void run(void)
{
    printf("IMU axis test.\n");
    printf("Hold flat: Z near 1 g. Tilt right: +X. Tilt forward: +Y.\n\n");

    if (!qmi8658_present()) {
        while (!demo_exit_requested()) {
            demo_show_message("NO IMU", "QMI8658 not detected", GFX_RED);
            demo_delay_ms(200);
        }
        return;
    }

    while (!demo_exit_requested()) {
        imu_sample_t sample;
        if (!demo_imu_latest(&sample)) {
            demo_delay_ms(20);
            continue;
        }
        const vector3f_t acc = sample.acc;
        const vector3f_t gyro = sample.gyro;
        const int16_t *acc_raw = sample.acc_raw;
        const int16_t *gyro_raw = sample.gyro_raw;

        demo_frame_begin(GFX_BLACK);
        draw_reference();

        draw_vector(acc.x, acc.y, acc.z, GFX_RED);
        draw_vector(gyro.x / 100.0f, gyro.y / 100.0f, gyro.z / 100.0f, GFX_GREEN);

        /* Magnitude bars, kept from the original, but grown out from the centre
         * column so they stay inside the round panel. */
        char line[64];
        const uint16_t bar_colors[3] = {GFX_RED, GFX_GREEN, GFX_BLUE};
        const float axes[3] = {acc.x, acc.y, acc.z};

        snprintf(line, sizeof(line), "temp %.1f" GFX_DEG "C",
                 (double)qmi8658_read_temperature());
        gfx_text_centered(DISP_CX, DEMO_ROW_TOP(0), line, GFX_DGREY, 2);

        for (int i = 0; i < 3; i++) {
            const int bar = (fabsf(axes[i]) > 0.1f) ? (int)(axes[i] * S(50)) : 0;
            gfx_fill_rect(DISP_CX, DEMO_ROW_TOP(1) + i * 8, bar, 5, bar_colors[i]);
        }

        snprintf(line, sizeof(line), "acc %+.2f %+.2f %+.2f g",
                 (double)acc.x, (double)acc.y, (double)acc.z);
        gfx_text_centered(DISP_CX, DEMO_ROW_BOTTOM(2), line, GFX_RED, 2);
        snprintf(line, sizeof(line), "gyro %+6.1f %+6.1f %+6.1f",
                 (double)gyro.x, (double)gyro.y, (double)gyro.z);
        gfx_text_centered(DISP_CX, DEMO_ROW_BOTTOM(1), line, GFX_GREEN, 2);
        snprintf(line, sizeof(line), "raw a %+6d %+6d %+6d  g %+6d %+6d %+6d",
                 acc_raw[0], acc_raw[1], acc_raw[2],
                 gyro_raw[0], gyro_raw[1], gyro_raw[2]);
        gfx_text_centered(DISP_CX, DEMO_ROW_BOTTOM(0), line, GFX_GREY, 1);
        demo_draw_exit_hint();

        demo_frame_end();

        printf("\rAccel: X:%+.2f Y:%+.2f Z:%+.2f | Gyro: X:%+6.1f Y:%+6.1f Z:%+6.1f  ",
               (double)acc.x, (double)acc.y, (double)acc.z,
               (double)gyro.x, (double)gyro.y, (double)gyro.z);

    }
    printf("\n");
}

const demo_t demo_axis_test = {
    .name = "axis_test",
    .summary = "live accelerometer and gyro vectors",
    .run = run,
};
