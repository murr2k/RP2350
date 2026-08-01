/**
 * Rotation axis test.
 *
 * Port of src/rotation_test.c. Each gyro axis gets its own dial so you can see
 * which physical rotation drives which channel. This is the tool to run first
 * when working out the sensor orientation on a new board.
 */

#include <math.h>
#include <stdio.h>

#include "demo_common.h"

static void draw_dial(int cx, int cy, float angle, uint16_t color, const char *label)
{
    gfx_circle(cx, cy, S(30), GFX_DGREY);

    const int x = cx + (int)(S(25) * cosf(angle));
    const int y = cy + (int)(S(25) * sinf(angle));
    gfx_arrow(cx, cy, x, y, color);

    gfx_fill_rect(cx - S(5), cy - S(40), S(10), 4, color);
    gfx_text_centered(cx, cy + S(34), label, color, 2);
    gfx_printf(cx - S(20), cy + S(46), color, 1, "%+7.1f" GFX_DEG,
               (double)(angle * RAD_TO_DEG));
}

static void draw_3d_axes(int cx, int cy)
{
    /* X right (red), Y up (green), Z out of the screen (blue, drawn diagonally). */
    gfx_arrow(cx, cy, cx + S(50), cy, GFX_RED);
    gfx_arrow(cx, cy, cx, cy - S(50), GFX_GREEN);
    gfx_arrow(cx, cy, cx - S(35), cy + S(35), GFX_BLUE);
}

static void run(void)
{
    printf("Rotation axis test.\n");
    printf("RED (X): roll, tilt left/right\n");
    printf("GREEN (Y): pitch, tilt forward/back\n");
    printf("BLUE (Z): yaw, rotate flat\n\n");

    if (!qmi8658_present()) {
        while (!demo_exit_requested()) {
            demo_show_message("NO IMU", "QMI8658 not detected", GFX_RED);
            demo_delay_ms(200);
        }
        return;
    }

    demo_show_message("CALIBRATING", "hold the board still", GFX_YELLOW);
    qmi8658_calibrate(100);
    printf("Calibration done\n");

    float roll_angle = 0.0f;
    float pitch_angle = 0.0f;
    float yaw_angle = 0.0f;
    uint64_t last_us = demo_micros();

    while (!demo_exit_requested()) {
        int16_t acc_raw[3];
        int16_t gyro_raw[3];
        vector3f_t gyro;

        if (!qmi8658_read_raw(acc_raw, gyro_raw) || !demo_read_imu(NULL, &gyro)) {
            demo_delay_ms(10);
            continue;
        }

        const float dt = demo_delta_seconds(&last_us);

        roll_angle += gyro.x * dt * DEG_TO_RAD;
        pitch_angle += gyro.y * dt * DEG_TO_RAD;
        yaw_angle += gyro.z * dt * DEG_TO_RAD;

        if (roll_angle > PI) {
            roll_angle -= TWO_PI;
        }
        if (roll_angle < -PI) {
            roll_angle += TWO_PI;
        }
        if (pitch_angle > PI) {
            pitch_angle -= TWO_PI;
        }
        if (pitch_angle < -PI) {
            pitch_angle += TWO_PI;
        }
        if (yaw_angle > PI) {
            yaw_angle -= TWO_PI;
        }
        if (yaw_angle < -PI) {
            yaw_angle += TWO_PI;
        }

        demo_frame_begin(GFX_BLACK);

        draw_3d_axes(DISP_CX, DISP_CY);
        draw_dial(140, 160, roll_angle, GFX_RED, "X");
        draw_dial(340, 160, pitch_angle, GFX_GREEN, "Y");
        draw_dial(DISP_CX, 290, yaw_angle, GFX_BLUE, "Z");

        /* Rate bars, clamped at 40 units as in the original. They grow out from
         * the centre column now so they stay inside the circle. */
        const float rates[3] = {gyro.x, gyro.y, gyro.z};
        const uint16_t colors[3] = {GFX_RED, GFX_GREEN, GFX_BLUE};
        for (int i = 0; i < 3; i++) {
            int bar = (int)(fabsf(rates[i]) * 2.0f);
            if (bar > 40) {
                bar = 40;
            }
            bar = (int)(bar * DISP_SCALE);
            if (rates[i] < -1.0f) {
                bar = -bar;
            } else if (rates[i] <= 1.0f) {
                bar = 0;
            }
            gfx_fill_rect(DISP_CX, DEMO_ROW_TOP(0) + i * 10, bar, 6, colors[i]);
        }

        char line[48];
        snprintf(line, sizeof(line), "dps %+6.1f %+6.1f %+6.1f",
                 (double)gyro.x, (double)gyro.y, (double)gyro.z);
        gfx_text_centered(DISP_CX, DEMO_ROW_BOTTOM(0), line, GFX_WHITE, 2);
        snprintf(line, sizeof(line), "raw %+6d %+6d %+6d",
                 gyro_raw[0], gyro_raw[1], gyro_raw[2]);
        gfx_text_centered(DISP_CX, 424, line, GFX_GREY, 1);
        demo_draw_exit_hint();

        demo_frame_end();

        printf("\rRaw: GX:%+6d GY:%+6d GZ:%+6d | dps: X:%+6.1f Y:%+6.1f Z:%+6.1f  ",
               gyro_raw[0], gyro_raw[1], gyro_raw[2],
               (double)gyro.x, (double)gyro.y, (double)gyro.z);

    }
    printf("\n");
}

const demo_t demo_rotation_test = {
    .name = "rotation_test",
    .summary = "per-axis gyro dials for calibration",
    .run = run,
};
