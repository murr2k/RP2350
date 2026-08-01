/**
 * Gravity locked cube.
 *
 * Port of src/gravity_locked_cube.c. The cube is counter-rotated by the
 * measured roll around the gravity vector, so it appears to stay level while
 * the board turns. A complementary filter blends the integrated gyro rate with
 * the angle the accelerometer sees.
 */

#include <math.h>
#include <stdio.h>

#include "demo_common.h"

#define CUBE_HALF   40.0f
#define CUBE_DIST   150.0f
#define CUBE_FOCAL  100.0f
#define ALPHA       0.98f       /* 0.98 trusts the gyro, as in the original */

static float calibrate(void)
{
    demo_show_message("CALIBRATING", "hold the board still", GFX_YELLOW);

    float ax_sum = 0.0f;
    float ay_sum = 0.0f;
    const int samples = 30;

    for (int i = 0; i < samples; i++) {
        vector3f_t acc;
        if (demo_read_imu(&acc, NULL)) {
            ax_sum += acc.x;
            ay_sum += acc.y;
        }
        demo_delay_ms(20);
    }

    const float phi0 = atan2f(ax_sum / samples, ay_sum / samples);
    printf("Calibration done: %.2f deg\n", (double)(phi0 * RAD_TO_DEG));
    return phi0;
}

static void run(void)
{
    printf("Gravity locked cube: the cube stays level while the board turns.\n");

    if (!qmi8658_present()) {
        while (!demo_exit_requested()) {
            demo_show_message("NO IMU", "QMI8658 not detected", GFX_RED);
            demo_delay_ms(200);
        }
        return;
    }

    const float phi_cal = calibrate();
    float phi = 0.0f;
    uint64_t last_us = demo_micros();

    while (!demo_exit_requested()) {
        vector3f_t acc;
        vector3f_t gyro;
        if (!demo_read_imu(&acc, &gyro)) {
            demo_delay_ms(10);
            continue;
        }

        const float dt = demo_delta_seconds(&last_us);

        /* Gyro in rad/s, as the original converted it. */
        const float gx = gyro.x * DEG_TO_RAD;
        const float gy = gyro.y * DEG_TO_RAD;
        const float gz = gyro.z * DEG_TO_RAD;

        float ax = acc.x;
        float ay = acc.y;
        float az = acc.z;
        const float mag = sqrtf(ax * ax + ay * ay + az * az);
        if (mag > 0.0f) {
            ax /= mag;
            ay /= mag;
            az /= mag;
        }

        /* Component of the rotation rate along gravity. */
        const float w_par = ax * gx + ay * gy + az * gz;
        const float phi_gyro = phi + w_par * dt;

        float phi_accel = atan2f(ax, ay) - phi_cal;
        while (phi_accel > PI) {
            phi_accel -= TWO_PI;
        }
        while (phi_accel < -PI) {
            phi_accel += TWO_PI;
        }

        phi = ALPHA * phi_gyro + (1.0f - ALPHA) * phi_accel;

        demo_frame_begin(GFX_BLACK);

        vertex_t rotated[8] = {
            {-CUBE_HALF, -CUBE_HALF, -CUBE_HALF}, { CUBE_HALF, -CUBE_HALF, -CUBE_HALF},
            { CUBE_HALF,  CUBE_HALF, -CUBE_HALF}, {-CUBE_HALF,  CUBE_HALF, -CUBE_HALF},
            {-CUBE_HALF, -CUBE_HALF,  CUBE_HALF}, { CUBE_HALF, -CUBE_HALF,  CUBE_HALF},
            { CUBE_HALF,  CUBE_HALF,  CUBE_HALF}, {-CUBE_HALF,  CUBE_HALF,  CUBE_HALF},
        };
        for (int i = 0; i < 8; i++) {
            rotate_z(&rotated[i], -phi);
        }

        gfx_circle(DISP_CX, DISP_CY, S(115), GFX_CYAN);
        demo_draw_cube(rotated, CUBE_DIST, CUBE_FOCAL);

        /* Yellow needle pointing where the filter thinks "down" is. */
        const int gx_end = DISP_CX + (int)(S(50) * sinf(phi));
        const int gy_end = DISP_CY - (int)(S(50) * cosf(phi));
        gfx_thick_line(DISP_CX, DISP_CY, gx_end, gy_end, 3, GFX_YELLOW);

        gfx_printf(S(12), S(12), GFX_WHITE, 2, "angle %+7.1f" GFX_DEG,
                   (double)(phi * RAD_TO_DEG));
        gfx_printf(S(12), S(200), GFX_GREY, 2, "acc %+.2f %+.2f %+.2f",
                   (double)ax, (double)ay, (double)az);
        demo_draw_exit_hint();

        demo_frame_end();

        printf("\rAngle: %6.1f | Acc: %.2f %.2f %.2f      ",
               (double)(phi * RAD_TO_DEG), (double)ax, (double)ay, (double)az);

        demo_delay_ms(50);
    }
    printf("\n");
}

const demo_t demo_gravity_locked_cube = {
    .name = "gravity_locked_cube",
    .summary = "cube stays level with gravity",
    .run = run,
};
