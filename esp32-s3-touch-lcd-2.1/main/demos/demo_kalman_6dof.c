/**
 * 6-DOF quaternion Kalman filter.
 *
 * Port of src/kalman_6dof.c, the headline demo of the RP2350 v2.0.0 release.
 * Gyro integration for the prediction step, an accelerometer correction with a
 * proportional gain, and a complementary filter running alongside for
 * comparison.
 *
 * The serial protocol is unchanged, so regression_test.py, monitor_axes.py and
 * the rest of the host tools in the repository root work against this build:
 *   d            toggle the CSV debug stream (DEBUG_START / DEBUG_STOP markers)
 *   p            print the current parameters
 *   P<X>=<value> set a parameter: K, I, F, S, B
 *   r m + - q a h as documented in the on-device help
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "demo_common.h"

#define CUBE_DIST  150.0f
#define CUBE_FOCAL 10000.0f     /* the original's 100 * 100 term, unit cube */

typedef struct {
    quaternion_t q;
    float gx_bias, gy_bias, gz_bias;
    float P[2][2];
    float Q_angle;
    float Q_bias;
    float R_measure;
} kalman_state_t;

static kalman_state_t s_kalman;
static int s_display_mode;
static float s_comp_pitch, s_comp_roll, s_comp_yaw;
static bool s_debug_stream;
static int s_debug_counter;

/* Tunables exposed over the serial console. */
static float s_kp_gain = 2.0f;
static float s_ki_gain = 0.0f;
static float s_gyro_filter_alpha = 0.98f;

static const vertex_t s_unit_cube[8] = {
    {-0.5f, -0.5f, -0.5f}, {0.5f, -0.5f, -0.5f},
    {0.5f, 0.5f, -0.5f},   {-0.5f, 0.5f, -0.5f},
    {-0.5f, -0.5f, 0.5f},  {0.5f, -0.5f, 0.5f},
    {0.5f, 0.5f, 0.5f},    {-0.5f, 0.5f, 0.5f},
};

static void kalman_init(kalman_state_t *k)
{
    quat_identity(&k->q);
    k->gx_bias = 0.0f;
    k->gy_bias = 0.0f;
    k->gz_bias = 0.0f;
    k->P[0][0] = 1.0f;
    k->P[0][1] = 0.0f;
    k->P[1][0] = 0.0f;
    k->P[1][1] = 1.0f;
    k->Q_angle = 0.001f;
    k->Q_bias = 0.003f;
    k->R_measure = 0.03f;
}

static void kalman_update_accel(kalman_state_t *k, float ax, float ay, float az, float dt)
{
    const float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm < 0.5f || norm > 2.0f) {
        return;     /* the board is being shaken, the accelerometer is useless */
    }
    ax /= norm;
    ay /= norm;
    az /= norm;

    float gx, gy, gz;
    quat_to_gravity(&k->q, &gx, &gy, &gz);

    const float ex = ay * gz - az * gy;
    const float ey = az * gx - ax * gz;
    const float ez = ax * gy - ay * gx;

    float accel_confidence = 1.0f - fabsf(norm - 1.0f);
    if (accel_confidence < 0.0f) {
        accel_confidence = 0.0f;
    }
    const float Kp = s_kp_gain * accel_confidence;

    /* The integral term stays off: bias is set at calibration time, and
     * letting it wander was what made the original drift. */
    const float max_bias = 0.2f;
    if (k->gx_bias > max_bias) {
        k->gx_bias = max_bias;
    }
    if (k->gx_bias < -max_bias) {
        k->gx_bias = -max_bias;
    }
    if (k->gy_bias > max_bias) {
        k->gy_bias = max_bias;
    }
    if (k->gy_bias < -max_bias) {
        k->gy_bias = -max_bias;
    }
    if (k->gz_bias > max_bias) {
        k->gz_bias = max_bias;
    }
    if (k->gz_bias < -max_bias) {
        k->gz_bias = -max_bias;
    }

    const float cx = Kp * ex;
    const float cy = Kp * ey;
    const float cz = Kp * ez;

    k->q.w -= dt * 0.5f * (k->q.x * cx + k->q.y * cy + k->q.z * cz);
    k->q.x += dt * 0.5f * (k->q.w * cx + k->q.y * cz - k->q.z * cy);
    k->q.y += dt * 0.5f * (k->q.w * cy - k->q.x * cz + k->q.z * cx);
    k->q.z += dt * 0.5f * (k->q.w * cz + k->q.x * cy - k->q.y * cx);

    quat_normalize(&k->q);
}

/* Everything the render core needs, filled by the sensor core under the lock so
 * a frame never shows a half updated filter. */
typedef struct {
    quaternion_t q;
    float pitch, roll, yaw;
    float comp_pitch, comp_roll, comp_yaw;
    float raw_g[3];         /* degrees per second, after the axis swap */
    float acc[3];
    float filt_g[3];        /* degrees per second */
    float bias[3];
} kalman_snapshot_t;

static kalman_snapshot_t s_snap;
static float s_filtered_g[3];

static void filter_step(const imu_sample_t *sample)
{
    const float ax = sample->acc.x;
    const float ay = sample->acc.y;
    const float az = sample->acc.z;

    /* Axis swap inherited from the RP2350 build. */
    const float gx = sample->gyro.y * DEG_TO_RAD;
    const float gy = -sample->gyro.x * DEG_TO_RAD;
    const float gz = -sample->gyro.z * DEG_TO_RAD;
    const float dt = sample->dt;
    const float alpha_f = s_gyro_filter_alpha;
    const float alpha_c = 0.98f;

    s_filtered_g[0] = alpha_f * s_filtered_g[0] + (1.0f - alpha_f) * gx;
    s_filtered_g[1] = alpha_f * s_filtered_g[1] + (1.0f - alpha_f) * gy;
    s_filtered_g[2] = alpha_f * s_filtered_g[2] + (1.0f - alpha_f) * gz;

    const float accel_pitch = atan2f(ax, sqrtf(ay * ay + az * az));
    const float accel_roll = atan2f(ay, sqrtf(ax * ax + az * az));

    demo_lock();

    const float corrected_gx = s_filtered_g[0] - s_kalman.gx_bias;
    const float corrected_gy = s_filtered_g[1] - s_kalman.gy_bias;
    const float corrected_gz = s_filtered_g[2] - s_kalman.gz_bias;

    quat_integrate(&s_kalman.q, corrected_gx, corrected_gy, corrected_gz, dt);
    kalman_update_accel(&s_kalman, ax, ay, az, dt);

    float roll, pitch, yaw;
    quat_to_euler(&s_kalman.q, &roll, &pitch, &yaw);

    s_comp_pitch = alpha_c * (s_comp_pitch + corrected_gx * dt) +
                   (1.0f - alpha_c) * accel_pitch;
    s_comp_roll = alpha_c * (s_comp_roll + corrected_gy * dt) +
                  (1.0f - alpha_c) * accel_roll;
    s_comp_yaw += corrected_gz * dt;
    if (s_comp_yaw > PI) {
        s_comp_yaw -= TWO_PI;
    }
    if (s_comp_yaw < -PI) {
        s_comp_yaw += TWO_PI;
    }

    s_snap.q = s_kalman.q;
    s_snap.pitch = pitch;
    s_snap.roll = roll;
    s_snap.yaw = yaw;
    s_snap.comp_pitch = s_comp_pitch;
    s_snap.comp_roll = s_comp_roll;
    s_snap.comp_yaw = s_comp_yaw;
    s_snap.raw_g[0] = gx * RAD_TO_DEG;
    s_snap.raw_g[1] = gy * RAD_TO_DEG;
    s_snap.raw_g[2] = gz * RAD_TO_DEG;
    s_snap.acc[0] = ax;
    s_snap.acc[1] = ay;
    s_snap.acc[2] = az;
    s_snap.filt_g[0] = s_filtered_g[0] * RAD_TO_DEG;
    s_snap.filt_g[1] = s_filtered_g[1] * RAD_TO_DEG;
    s_snap.filt_g[2] = s_filtered_g[2] * RAD_TO_DEG;
    s_snap.bias[0] = s_kalman.gx_bias;
    s_snap.bias[1] = s_kalman.gy_bias;
    s_snap.bias[2] = s_kalman.gz_bias;

    demo_unlock();
}

static void draw_angle_bar(int x, int y, float angle, uint16_t color)
{
    const int width = S(80);
    const int pos = x + (int)(angle * RAD_TO_DEG * width / 180.0f);

    gfx_hline(x - width / 2, y, width, GFX_DGREY);
    if (pos >= x - width / 2 && pos <= x + width / 2) {
        gfx_fill_rect(pos - 2, y - S(3), 5, S(6), color);
    }
    gfx_vline(x, y - S(5), S(10), GFX_WHITE);
}

static void print_help(void)
{
    printf("\n=== 6-DOF Kalman Controls ===\n");
    printf("r: Reset orientation\n");
    printf("m: Switch display mode\n");
    printf("d: Toggle debug streaming\n");
    printf("p: Print current parameters\n");
    printf("+/-: Adjust R_measure (accel trust)\n");
    printf("q/a: Adjust Q_angle (process noise)\n");
    printf("P<X>=<val>: Set parameter (K,I,F,S,B)\n");
    printf("ESC or ~: back to the launcher menu\n");
    printf("==============================\n");
}

static void process_command(char c)
{
    static bool parse_mode = false;
    static char parse_buffer[32];
    static int parse_index = 0;

    if (c == 'P' && !parse_mode) {
        parse_mode = true;
        parse_index = 0;
        return;
    }

    if (parse_mode) {
        if (c == '\n' || c == '\r') {
            parse_buffer[parse_index] = '\0';

            char param;
            float value;
            if (sscanf(parse_buffer, "%c=%f", &param, &value) == 2) {
                switch (param) {
                case 'K':
                    s_kp_gain = value;
                    printf("Set Kp_gain=%.3f\n", (double)s_kp_gain);
                    break;
                case 'I':
                    s_ki_gain = value;
                    printf("Set Ki_gain=%.5f\n", (double)s_ki_gain);
                    break;
                case 'F':
                    s_gyro_filter_alpha = value;
                    printf("Set filter_alpha=%.3f\n", (double)s_gyro_filter_alpha);
                    break;
                case 'S':
                    printf("Scale is fixed at 128 LSB/dps for QMI8658\n");
                    break;
                case 'B': {
                    float bx, by, bz;
                    if (sscanf(parse_buffer, "B=%f,%f,%f", &bx, &by, &bz) == 3) {
                        demo_lock();
                        s_kalman.gx_bias = bx;
                        s_kalman.gy_bias = by;
                        s_kalman.gz_bias = bz;
                        demo_unlock();
                        printf("Set bias=[%.4f,%.4f,%.4f]\n",
                               (double)bx, (double)by, (double)bz);
                    }
                    break;
                }
                default:
                    break;
                }
            }
            parse_mode = false;
        } else if (parse_index < (int)sizeof(parse_buffer) - 1) {
            parse_buffer[parse_index++] = c;
        }
        return;
    }

    switch (c) {
    case 'r': {
        demo_lock();
        const float bx = s_kalman.gx_bias;
        const float by = s_kalman.gy_bias;
        const float bz = s_kalman.gz_bias;

        quat_identity(&s_kalman.q);
        s_kalman.gx_bias = bx;
        s_kalman.gy_bias = by;
        s_kalman.gz_bias = bz;
        s_comp_pitch = 0.0f;
        s_comp_roll = 0.0f;
        s_comp_yaw = 0.0f;
        demo_unlock();

        printf("Reset orientation (bias preserved: %.4f,%.4f,%.4f)\n",
               (double)bx, (double)by, (double)bz);
        break;
    }
    case 'm':
        s_display_mode = (s_display_mode + 1) % 3;
        printf("Mode: %d (0=Kalman, 1=Comp, 2=Both)\n", s_display_mode);
        break;
    case '+':
        s_kalman.R_measure += 0.01f;
        printf("R_measure: %.3f (trust accel less)\n", (double)s_kalman.R_measure);
        break;
    case '-':
        s_kalman.R_measure -= 0.01f;
        if (s_kalman.R_measure < 0.001f) {
            s_kalman.R_measure = 0.001f;
        }
        printf("R_measure: %.3f (trust accel more)\n", (double)s_kalman.R_measure);
        break;
    case 'q':
        s_kalman.Q_angle += 0.001f;
        printf("Q_angle: %.4f\n", (double)s_kalman.Q_angle);
        break;
    case 'a':
        s_kalman.Q_angle -= 0.001f;
        if (s_kalman.Q_angle < 0.0001f) {
            s_kalman.Q_angle = 0.0001f;
        }
        printf("Q_angle: %.4f\n", (double)s_kalman.Q_angle);
        break;
    case 'd':
        s_debug_stream = !s_debug_stream;
        if (s_debug_stream) {
            printf("DEBUG_START\n");
            printf("# CSV format: time_ms,raw_gx,raw_gy,raw_gz,raw_ax,raw_ay,raw_az,");
            printf("filt_gx,filt_gy,filt_gz,bias_x,bias_y,bias_z,");
            printf("qw,qx,qy,qz,pitch,roll,yaw,Kp,Ki,filter\n");
        } else {
            printf("DEBUG_STOP\n");
        }
        break;
    case 'p':
        printf("PARAMS: Kp=%.3f Filter=%.3f ", (double)s_kp_gain,
               (double)s_gyro_filter_alpha);
        printf("Bias=[%.4f,%.4f,%.4f] ", (double)s_kalman.gx_bias,
               (double)s_kalman.gy_bias, (double)s_kalman.gz_bias);
        printf("Q_angle=%.4f Q_bias=%.4f R=%.3f\n", (double)s_kalman.Q_angle,
               (double)s_kalman.Q_bias, (double)s_kalman.R_measure);
        break;
    case 'h':
        print_help();
        break;
    default:
        break;
    }
}

static void run(void)
{
    printf("6-DOF Quaternion Kalman Filter\n");

    if (!qmi8658_present()) {
        while (!demo_exit_requested()) {
            demo_show_message("NO IMU", "QMI8658 not detected", GFX_RED);
            demo_delay_ms(200);
        }
        return;
    }

    kalman_init(&s_kalman);
    s_display_mode = 0;
    s_debug_stream = false;
    s_comp_pitch = s_comp_roll = s_comp_yaw = 0.0f;

    /* Same choice as the original: start from zero bias and let the operator
     * apply a measured one with PB=... rather than trusting a rushed average. */
    printf("Note: starting with zero bias. Use PB=x,y,z to apply a calibration.\n");
    printf("Press 'h' for help\n");

    s_filtered_g[0] = s_filtered_g[1] = s_filtered_g[2] = 0.0f;
    demo_set_filter(filter_step);

    while (!demo_exit_requested()) {
        int c;
        while ((c = demo_read_char()) >= 0) {
            process_command((char)c);
        }

        kalman_snapshot_t snap;
        demo_lock();
        snap = s_snap;
        demo_unlock();

        const float kalman_pitch = snap.pitch;
        const float kalman_roll = snap.roll;
        const float kalman_yaw = snap.yaw;

        demo_frame_begin(GFX_BLACK);

        if (s_display_mode == 0 || s_display_mode == 2) {
            vertex_t cube[8];
            for (int i = 0; i < 8; i++) {
                cube[i] = s_unit_cube[i];
                quat_rotate_vertex(&cube[i], &snap.q);
            }
            demo_draw_cube_mono(cube, CUBE_DIST, CUBE_FOCAL, GFX_GREEN);
            draw_angle_bar(DISP_CX, 340, kalman_pitch, GFX_GREEN);
            draw_angle_bar(DISP_CX, 360, kalman_roll, GFX_GREEN);
        }

        if (s_display_mode == 1 || s_display_mode == 2) {
            vertex_t cube[8];
            for (int i = 0; i < 8; i++) {
                cube[i] = s_unit_cube[i];
                if (s_display_mode == 2) {
                    cube[i].x *= 0.5f;
                    cube[i].y *= 0.5f;
                    cube[i].z *= 0.5f;
                }

                const float cx = cube[i].x;
                const float cy = cube[i].y;
                const float cz = cube[i].z;

                const float y1 = cy * cosf(snap.comp_pitch) - cz * sinf(snap.comp_pitch);
                const float z1 = cy * sinf(snap.comp_pitch) + cz * cosf(snap.comp_pitch);
                const float x2 = cx * cosf(snap.comp_roll) + z1 * sinf(snap.comp_roll);
                const float z2 = -cx * sinf(snap.comp_roll) + z1 * cosf(snap.comp_roll);

                cube[i].x = x2 * cosf(snap.comp_yaw) - y1 * sinf(snap.comp_yaw);
                cube[i].y = x2 * sinf(snap.comp_yaw) + y1 * cosf(snap.comp_yaw);
                cube[i].z = z2;
            }
            demo_draw_cube_mono(cube, CUBE_DIST, CUBE_FOCAL, GFX_RED);

            if (s_display_mode == 1) {
                draw_angle_bar(DISP_CX, 340, snap.comp_pitch, GFX_RED);
                draw_angle_bar(DISP_CX, 360, snap.comp_roll, GFX_RED);
            }
        }

        static const char *mode_names[3] = {"KALMAN", "COMPLEMENTARY", "BOTH"};
        char line[48];

        snprintf(line, sizeof(line), "mode %s", mode_names[s_display_mode]);
        gfx_text_centered(DISP_CX, DEMO_ROW_TOP(0), line, GFX_WHITE, 2);
        snprintf(line, sizeof(line), "P %+6.1f R %+6.1f Y %+6.1f",
                 (double)(kalman_pitch * RAD_TO_DEG), (double)(kalman_roll * RAD_TO_DEG),
                 (double)(kalman_yaw * RAD_TO_DEG));
        gfx_text_centered(DISP_CX, DEMO_ROW_TOP(1), line, GFX_GREEN, 2);
        snprintf(line, sizeof(line), "Kp %.2f  f %.2f %s",
                 (double)s_kp_gain, (double)s_gyro_filter_alpha,
                 s_debug_stream ? "CSV" : "");
        gfx_text_centered(DISP_CX, DEMO_ROW_BOTTOM(0), line, GFX_DGREY, 2);
        demo_draw_exit_hint();

        demo_frame_end();

        if (s_debug_stream && ++s_debug_counter >= 5) {
            s_debug_counter = 0;
            printf("%lu,", (unsigned long)demo_millis());
            printf("%.3f,%.3f,%.3f,", (double)snap.raw_g[0], (double)snap.raw_g[1],
                   (double)snap.raw_g[2]);
            printf("%.3f,%.3f,%.3f,", (double)snap.acc[0], (double)snap.acc[1],
                   (double)snap.acc[2]);
            printf("%.3f,%.3f,%.3f,", (double)snap.filt_g[0], (double)snap.filt_g[1],
                   (double)snap.filt_g[2]);
            printf("%.4f,%.4f,%.4f,", (double)snap.bias[0], (double)snap.bias[1],
                   (double)snap.bias[2]);
            printf("%.4f,%.4f,%.4f,%.4f,", (double)snap.q.w, (double)snap.q.x,
                   (double)snap.q.y, (double)snap.q.z);
            printf("%.2f,%.2f,%.2f,", (double)(kalman_pitch * RAD_TO_DEG),
                   (double)(kalman_roll * RAD_TO_DEG), (double)(kalman_yaw * RAD_TO_DEG));
            printf("%.3f,%.5f,%.3f\n", (double)s_kp_gain, (double)s_ki_gain,
                   (double)s_gyro_filter_alpha);
        }

    }

    demo_set_filter(NULL);

    if (s_debug_stream) {
        printf("DEBUG_STOP\n");
        s_debug_stream = false;
    }
}

const demo_t demo_kalman_6dof = {
    .name = "kalman_6dof",
    .summary = "quaternion Kalman filter, CSV stream",
    .run = run,
};
