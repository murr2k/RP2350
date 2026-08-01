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
        int16_t acc_raw[3];
        int16_t gyro_raw[3];
        vector3f_t acc;
        vector3f_t gyro;

        if (!qmi8658_read_raw(acc_raw, gyro_raw) || !demo_read_imu(&acc, &gyro)) {
            demo_delay_ms(20);
            continue;
        }

        demo_frame_begin(GFX_BLACK);
        draw_reference();

        draw_vector(acc.x, acc.y, acc.z, GFX_RED);
        draw_vector(gyro.x / 100.0f, gyro.y / 100.0f, gyro.z / 100.0f, GFX_GREEN);

        /* Magnitude bars, kept from the original. */
        const int ax_bar = (int)(fabsf(acc.x) * S(50));
        const int ay_bar = (int)(fabsf(acc.y) * S(50));
        const int az_bar = (int)(fabsf(acc.z) * S(50));
        if (acc.x > 0.1f) {
            gfx_fill_rect(S(10), S(10), ax_bar, 4, GFX_RED);
        } else if (acc.x < -0.1f) {
            gfx_fill_rect(S(10), S(16), ax_bar, 4, GFX_RGB(255, 128, 128));
        }
        if (acc.y > 0.1f) {
            gfx_fill_rect(S(230), S(10), -ay_bar, 4, GFX_GREEN);
        } else if (acc.y < -0.1f) {
            gfx_fill_rect(S(230), S(16), -ay_bar, 4, GFX_RGB(128, 255, 128));
        }
        if (fabsf(acc.z) > 0.1f) {
            gfx_fill_rect(DISP_CX - az_bar / 2, (acc.z > 0) ? S(230) : S(224),
                          az_bar, 4, GFX_BLUE);
        }

        gfx_printf(S(10), S(196), GFX_RED, 2, "acc  %+.2f %+.2f %+.2f g",
                   (double)acc.x, (double)acc.y, (double)acc.z);
        gfx_printf(S(10), S(208), GFX_GREEN, 2, "gyro %+6.1f %+6.1f %+6.1f dps",
                   (double)gyro.x, (double)gyro.y, (double)gyro.z);
        gfx_printf(S(10), S(220), GFX_GREY, 1, "raw a %+6d %+6d %+6d   g %+6d %+6d %+6d",
                   acc_raw[0], acc_raw[1], acc_raw[2],
                   gyro_raw[0], gyro_raw[1], gyro_raw[2]);
        gfx_printf(S(10), S(228), GFX_DGREY, 1, "temp %.1f" GFX_DEG "C",
                   (double)qmi8658_read_temperature());
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
