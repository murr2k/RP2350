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
/* The original blended with a fixed 0.98 at about 20 Hz, which is a 2.45 s time
 * constant. Derive the blend from dt now that the filter runs at 250 Hz, so the
 * balance between gyro and accelerometer stays where it was. */
#define FILTER_TAU  2.45f

/* Written on the sensor core, read on the render core. */
static float s_phi;
static float s_phi_cal;
static vector3f_t s_gravity;

static void filter_step(const imu_sample_t *sample)
{
    float ax = sample->acc.x;
    float ay = sample->acc.y;
    float az = sample->acc.z;

    const float mag = sqrtf(ax * ax + ay * ay + az * az);
    if (mag > 0.0f) {
        ax /= mag;
        ay /= mag;
        az /= mag;
    }

    /* Component of the rotation rate along gravity. */
    const float w_par = ax * sample->gyro.x * DEG_TO_RAD +
                        ay * sample->gyro.y * DEG_TO_RAD +
                        az * sample->gyro.z * DEG_TO_RAD;

    float phi;
    float phi_cal;
    demo_lock();
    phi = s_phi;
    phi_cal = s_phi_cal;
    demo_unlock();

    const float phi_gyro = phi + w_par * sample->dt;

    float phi_accel = atan2f(ax, ay) - phi_cal;
    while (phi_accel > PI) {
        phi_accel -= TWO_PI;
    }
    while (phi_accel < -PI) {
        phi_accel += TWO_PI;
    }

    const float alpha = FILTER_TAU / (FILTER_TAU + sample->dt);
    phi = alpha * phi_gyro + (1.0f - alpha) * phi_accel;

    demo_lock();
    s_phi = phi;
    s_gravity.x = ax;
    s_gravity.y = ay;
    s_gravity.z = az;
    demo_unlock();
}

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

    s_phi = 0.0f;
    s_phi_cal = calibrate();
    demo_set_filter(filter_step);

    while (!demo_exit_requested()) {
        float phi;
        float ax, ay, az;
        demo_lock();
        phi = s_phi;
        ax = s_gravity.x;
        ay = s_gravity.y;
        az = s_gravity.z;
        demo_unlock();

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

        char line[48];
        snprintf(line, sizeof(line), "angle %+7.1f" GFX_DEG, (double)(phi * RAD_TO_DEG));
        gfx_text_centered(DISP_CX, DEMO_ROW_TOP(0), line, GFX_WHITE, 2);

        snprintf(line, sizeof(line), "acc %+.2f %+.2f %+.2f",
                 (double)ax, (double)ay, (double)az);
        gfx_text_centered(DISP_CX, DEMO_ROW_BOTTOM(0), line, GFX_GREY, 2);
        demo_draw_exit_hint();

        demo_frame_end();

        printf("\rAngle: %6.1f | Acc: %.2f %.2f %.2f      ",
               (double)(phi * RAD_TO_DEG), (double)ax, (double)ay, (double)az);

    }
    demo_set_filter(NULL);
    printf("\n");
}

const demo_t demo_gravity_locked_cube = {
    .name = "gravity_locked_cube",
    .summary = "cube stays level with gravity",
    .run = run,
};
