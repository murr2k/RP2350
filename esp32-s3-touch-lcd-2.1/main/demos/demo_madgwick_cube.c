/**
 * Madgwick 6-DOF cube.
 *
 * Port of src/madgwick_cube.c. Gradient descent sensor fusion over the
 * accelerometer and gyroscope gives a drift free orientation, drawn as a cube
 * plus a body axis triad.
 */

#include <math.h>
#include <stdio.h>

#include "demo_common.h"

#define CUBE_DIST  200.0f
#define CUBE_FOCAL 120.0f
#define BETA       0.1f
#define CAL_SAMPLES 200

static void draw_axes(const quaternion_t *q)
{
    vertex_t axes[3] = {{40, 0, 0}, {0, 40, 0}, {0, 0, 40}};
    const uint16_t colors[3] = {GFX_RED, GFX_GREEN, GFX_BLUE};

    int ox, oy;
    demo_project((vertex_t){0, 0, 0}, CUBE_DIST, CUBE_FOCAL, &ox, &oy);

    for (int i = 0; i < 3; i++) {
        quat_rotate_vertex(&axes[i], q);
        int x, y;
        demo_project(axes[i], CUBE_DIST, CUBE_FOCAL, &x, &y);
        gfx_thick_line(ox, oy, x, y, 3, colors[i]);
    }
}

static void run(void)
{
    printf("Madgwick 6-DOF cube. Full orientation tracking.\n");

    if (!qmi8658_present()) {
        while (!demo_exit_requested()) {
            demo_show_message("NO IMU", "QMI8658 not detected", GFX_RED);
            demo_delay_ms(200);
        }
        return;
    }

    demo_show_message("CALIBRATING", "hold still for 2 seconds", GFX_YELLOW);
    printf("Calibrating gyro, hold still...\n");

    /* The RP2350 version averaged the raw rate itself. The driver has the same
     * job built in now, so the offset ends up inside qmi8658_read(). */
    qmi8658_calibrate(CAL_SAMPLES);
    const vector3f_t offset = qmi8658_gyro_offset();
    printf("Calibration complete: %.3f %.3f %.3f dps\n",
           (double)offset.x, (double)offset.y, (double)offset.z);

    quaternion_t q;
    quat_identity(&q);
    uint64_t last_us = demo_micros();
    uint32_t frames = 0;

    while (!demo_exit_requested()) {
        vector3f_t acc;
        vector3f_t gyro;
        if (!demo_read_imu(&acc, &gyro)) {
            demo_delay_ms(10);
            continue;
        }

        /* The original inverted the Y rate after its own axis testing. */
        const float gx = gyro.x * DEG_TO_RAD;
        const float gy = -gyro.y * DEG_TO_RAD;
        const float gz = gyro.z * DEG_TO_RAD;

        const float dt = demo_delta_seconds(&last_us);
        madgwick_update(&q, BETA, gx, gy, gz, acc.x, acc.y, acc.z, dt);

        float roll, pitch, yaw;
        quat_to_euler(&q, &roll, &pitch, &yaw);

        demo_frame_begin(GFX_BLACK);

        vertex_t rotated[8];
        for (int i = 0; i < 8; i++) {
            rotated[i] = demo_cube_vertices[i];
            quat_rotate_vertex(&rotated[i], &q);
        }

        draw_axes(&q);
        demo_draw_cube(rotated, CUBE_DIST, CUBE_FOCAL);

        gfx_printf(S(10), S(10), GFX_WHITE, 2, "R %+6.1f" GFX_DEG, (double)(roll * RAD_TO_DEG));
        gfx_printf(S(10), S(22), GFX_WHITE, 2, "P %+6.1f" GFX_DEG, (double)(pitch * RAD_TO_DEG));
        gfx_printf(S(10), S(34), GFX_WHITE, 2, "Y %+6.1f" GFX_DEG, (double)(yaw * RAD_TO_DEG));
        gfx_printf(S(10), S(200), GFX_GREY, 2, "q %.2f %.2f %.2f %.2f",
                   (double)q.w, (double)q.x, (double)q.y, (double)q.z);
        gfx_printf(S(10), S(212), GFX_DGREY, 2, "beta %.2f", (double)BETA);
        demo_draw_exit_hint();

        demo_frame_end();

        if ((++frames % 10) == 0) {
            printf("\rR:%6.1f P:%6.1f Y:%6.1f | Q: %.2f %.2f %.2f %.2f   ",
                   (double)(roll * RAD_TO_DEG), (double)(pitch * RAD_TO_DEG),
                   (double)(yaw * RAD_TO_DEG),
                   (double)q.w, (double)q.x, (double)q.y, (double)q.z);
        }

    }
    printf("\n");
}

const demo_t demo_madgwick_cube = {
    .name = "madgwick_cube",
    .summary = "6-DOF Madgwick orientation",
    .run = run,
};
