/**
 * 6-DOF Kalman filter with runtime axis configuration and data injection.
 *
 * Port of src/kalman_6dof_configurable.c. Same serial grammar as the RP2350
 * build, which is what the regression harness in the repository root drives:
 *
 *   AG<n>=<+-1>   flip gyro axis n            AA<n>=<+-1>  flip accel axis n
 *   AS<n>=<scale> scale gyro axis n           AW<n>=<w>    accel weight axis n
 *   AMAP=a,b,c    remap the gyro axes
 *   TX=30 TY=20 TZ=10 TA=1 TB=1 TC=1 TM=45 T0=0   built-in test signals
 *   PK=<v> PI=<v> PF=<v> PB=<x>,<y>,<z>       filter parameters
 *   I<ax>,<ay>,<az>,<gx>,<gy>,<gz>            inject a sample, echo CSV
 *   r reset   d debug stream   p print config   h help
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "demo_common.h"

#define CUBE_DIST  1.5f         /* unit cube, camera 1.5 units away */
#define CUBE_FOCAL 100.0f

typedef struct {
    int gyro_map[3];
    int accel_map[3];
    float gyro_sign[3];
    float accel_sign[3];
    float gyro_scale[3];
    float accel_scale[3];
    float accel_weight[3];
} axis_config_t;

typedef enum {
    TEST_OFF = 0,
    TEST_GYRO_X,
    TEST_GYRO_Y,
    TEST_GYRO_Z,
    TEST_ACCEL_X,
    TEST_ACCEL_Y,
    TEST_ACCEL_Z,
    TEST_COMBINED,
} test_mode_t;

static axis_config_t s_axis;
static test_mode_t s_test_mode;
static float s_test_amplitude;
static uint64_t s_test_start_us;

static quaternion_t s_q;
static float s_gx_bias, s_gy_bias, s_gz_bias;
static bool s_debug_stream;

static float s_kp_gain = 2.0f;
static float s_ki_gain = 0.001f;
static float s_gyro_filter_alpha = 0.98f;

static const vertex_t s_unit_cube[8] = {
    {-0.5f, -0.5f, -0.5f}, {0.5f, -0.5f, -0.5f},
    {0.5f, 0.5f, -0.5f},   {-0.5f, 0.5f, -0.5f},
    {-0.5f, -0.5f, 0.5f},  {0.5f, -0.5f, 0.5f},
    {0.5f, 0.5f, 0.5f},    {-0.5f, 0.5f, 0.5f},
};

static void axis_defaults(void)
{
    s_axis = (axis_config_t){
        .gyro_map = {1, 0, 2},                      /* swap X and Y */
        .accel_map = {1, 0, 2},
        .gyro_sign = {1.0f, -1.0f, -1.0f},
        .accel_sign = {1.0f, -1.0f, 1.0f},
        .gyro_scale = {1.0f, 1.0f, 1.0f},
        .accel_scale = {1.0f, 1.0f, 1.0f},
        .accel_weight = {1.0f, 1.0f, 1.0f},
    };
}

static void apply_axis_config(const float raw[3], float out[3], const int map[3],
                              const float sign[3], const float scale[3])
{
    for (int i = 0; i < 3; i++) {
        out[i] = raw[map[i]] * sign[i] * scale[i];
    }
}

static void apply_test_signal(float *gx, float *gy, float *gz,
                              float *ax, float *ay, float *az)
{
    if (s_test_mode == TEST_OFF) {
        return;
    }

    const float elapsed = (float)(demo_micros() - s_test_start_us) / 1000000.0f;
    const float signal = s_test_amplitude * sinf(elapsed * TWO_PI * 0.5f);   /* 0.5 Hz */

    switch (s_test_mode) {
    case TEST_OFF:
        return;
    case TEST_GYRO_X:
        *gx += signal;
        break;
    case TEST_GYRO_Y:
        *gy += signal;
        break;
    case TEST_GYRO_Z:
        *gz += signal;
        break;
    case TEST_ACCEL_X:
        *ax += signal * 0.1f;
        break;
    case TEST_ACCEL_Y:
        *ay += signal * 0.1f;
        break;
    case TEST_ACCEL_Z:
        *az += signal * 0.1f;
        break;
    case TEST_COMBINED:
        *gx += signal * 0.5f;
        *gy += signal * 0.3f;
        *gz += signal * 0.2f;
        break;
    }
}

static void print_help(void)
{
    printf("\n=== AXIS CONFIGURATION HELP ===\n");
    printf("Axis commands:\n");
    printf("  AG0=-1  : Flip gyro X axis polarity\n");
    printf("  AG1=1   : Set gyro Y axis normal polarity\n");
    printf("  AA2=-1  : Flip accel Z axis polarity\n");
    printf("  AS0=1.5 : Scale gyro X by 1.5x\n");
    printf("  AW1=0.5 : Reduce accel Y weight to 0.5\n");
    printf("  AMAP=1,0,2 : Remap axes (swap X/Y)\n");
    printf("\nTest commands:\n");
    printf("  TX=30   : Test gyro X with 30 degree amplitude\n");
    printf("  TY=20   : Test gyro Y with 20 degree amplitude\n");
    printf("  TZ=10   : Test gyro Z with 10 degree amplitude\n");
    printf("  TA=1    : Test accel X\n");
    printf("  TM=45   : Test combined motion\n");
    printf("  T0=0    : Stop testing\n");
    printf("\nRegression testing:\n");
    printf("  I<ax>,<ay>,<az>,<gx>,<gy>,<gz> : Inject IMU data\n");
    printf("    Example: I0.0,0.0,1.0,30.0,0.0,0.0\n");
    printf("\nOther: r=reset, d=debug, p=print config, ESC/~=exit\n");
}

static void print_config(void)
{
    printf("AXIS CONFIG:\n");
    printf("  Gyro map: [%d,%d,%d] sign: [%.0f,%.0f,%.0f] scale: [%.2f,%.2f,%.2f]\n",
           s_axis.gyro_map[0], s_axis.gyro_map[1], s_axis.gyro_map[2],
           (double)s_axis.gyro_sign[0], (double)s_axis.gyro_sign[1],
           (double)s_axis.gyro_sign[2],
           (double)s_axis.gyro_scale[0], (double)s_axis.gyro_scale[1],
           (double)s_axis.gyro_scale[2]);
    printf("  Accel map: [%d,%d,%d] sign: [%.0f,%.0f,%.0f]\n",
           s_axis.accel_map[0], s_axis.accel_map[1], s_axis.accel_map[2],
           (double)s_axis.accel_sign[0], (double)s_axis.accel_sign[1],
           (double)s_axis.accel_sign[2]);
    printf("  Accel weight: [%.2f,%.2f,%.2f]\n",
           (double)s_axis.accel_weight[0], (double)s_axis.accel_weight[1],
           (double)s_axis.accel_weight[2]);
    printf("  Kp=%.3f Ki=%.5f filter=%.3f bias=[%.4f,%.4f,%.4f]\n",
           (double)s_kp_gain, (double)s_ki_gain, (double)s_gyro_filter_alpha,
           (double)s_gx_bias, (double)s_gy_bias, (double)s_gz_bias);
    printf("  Test mode: %d, amplitude: %.1f deg\n", (int)s_test_mode,
           (double)(s_test_amplitude * RAD_TO_DEG));
}

static void handle_injection(const char *body)
{
    float ax, ay, az, gx, gy, gz;
    if (sscanf(body, "%f,%f,%f,%f,%f,%f", &ax, &ay, &az, &gx, &gy, &gz) != 6) {
        return;
    }

    static uint64_t last_inject_us;
    const uint64_t now = demo_micros();
    float dt = 0.02f;
    if (last_inject_us != 0) {
        dt = (float)(now - last_inject_us) / 1000000.0f;
    }
    last_inject_us = now;

    const float test_gx = gx * DEG_TO_RAD - s_gx_bias;
    const float test_gy = gy * DEG_TO_RAD - s_gy_bias;
    const float test_gz = gz * DEG_TO_RAD - s_gz_bias;

    quat_integrate(&s_q, test_gx, test_gy, test_gz, dt);

    float roll, pitch, yaw;
    quat_to_euler(&s_q, &roll, &pitch, &yaw);

    /* Field order is the RP2350 wire format, which labelled the atan2 angle
     * "pitch" and the asin angle "roll" (the opposite way round from
     * kalman_6dof.c). Kept as it was so the host scripts see what they expect. */
    printf("CSV,%lu,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\n",
           (unsigned long)(now / 1000ULL),
           (double)ax, (double)ay, (double)az,
           (double)gx, (double)gy, (double)gz,
           (double)(roll * RAD_TO_DEG), (double)(pitch * RAD_TO_DEG),
           (double)(yaw * RAD_TO_DEG));
}

static void handle_axis_command(const char *body)
{
    char axis_cmd;
    int axis;
    float value;

    if (sscanf(body, "%c%d=%f", &axis_cmd, &axis, &value) == 3 && axis >= 0 && axis < 3) {
        switch (axis_cmd) {
        case 'G':
            s_axis.gyro_sign[axis] = (value < 0) ? -1.0f : 1.0f;
            printf("Gyro axis %d sign = %.0f\n", axis, (double)s_axis.gyro_sign[axis]);
            return;
        case 'A':
            s_axis.accel_sign[axis] = (value < 0) ? -1.0f : 1.0f;
            printf("Accel axis %d sign = %.0f\n", axis, (double)s_axis.accel_sign[axis]);
            return;
        case 'S':
            s_axis.gyro_scale[axis] = value;
            printf("Gyro axis %d scale = %.2f\n", axis, (double)value);
            return;
        case 'W':
            s_axis.accel_weight[axis] = value;
            printf("Accel axis %d weight = %.2f\n", axis, (double)value);
            return;
        default:
            break;
        }
    }

    int m0, m1, m2;
    if (sscanf(body, "MAP=%d,%d,%d", &m0, &m1, &m2) == 3) {
        s_axis.gyro_map[0] = m0;
        s_axis.gyro_map[1] = m1;
        s_axis.gyro_map[2] = m2;
        printf("Gyro map: X->%d Y->%d Z->%d\n", m0, m1, m2);
    }
}

static void handle_test_command(const char *body)
{
    char test_cmd;
    float amp;
    if (sscanf(body, "%c=%f", &test_cmd, &amp) != 2) {
        return;
    }

    s_test_amplitude = amp * DEG_TO_RAD;
    s_test_start_us = demo_micros();

    switch (test_cmd) {
    case 'X': s_test_mode = TEST_GYRO_X;  printf("Testing Gyro X, amplitude %.1f\n", (double)amp); break;
    case 'Y': s_test_mode = TEST_GYRO_Y;  printf("Testing Gyro Y, amplitude %.1f\n", (double)amp); break;
    case 'Z': s_test_mode = TEST_GYRO_Z;  printf("Testing Gyro Z, amplitude %.1f\n", (double)amp); break;
    case 'A': s_test_mode = TEST_ACCEL_X; printf("Testing Accel X\n"); break;
    case 'B': s_test_mode = TEST_ACCEL_Y; printf("Testing Accel Y\n"); break;
    case 'C': s_test_mode = TEST_ACCEL_Z; printf("Testing Accel Z\n"); break;
    case 'M': s_test_mode = TEST_COMBINED; printf("Testing combined motion\n"); break;
    case '0': s_test_mode = TEST_OFF;     printf("Test mode OFF\n"); break;
    default: break;
    }
}

static void handle_param_command(const char *body)
{
    char param;
    float value;
    if (sscanf(body, "%c=%f", &param, &value) != 2) {
        return;
    }

    switch (param) {
    case 'K':
        s_kp_gain = value;
        printf("Set Kp_gain=%.3f\n", (double)value);
        break;
    case 'I':
        s_ki_gain = value;
        printf("Set Ki_gain=%.5f\n", (double)value);
        break;
    case 'F':
        s_gyro_filter_alpha = value;
        printf("Set filter_alpha=%.3f\n", (double)value);
        break;
    case 'B': {
        float bx, by, bz;
        if (sscanf(body, "B=%f,%f,%f", &bx, &by, &bz) == 3) {
            s_gx_bias = bx;
            s_gy_bias = by;
            s_gz_bias = bz;
            printf("Set bias=[%.4f,%.4f,%.4f]\n", (double)bx, (double)by, (double)bz);
        }
        break;
    }
    default:
        break;
    }
}

static void process_command(char c)
{
    static bool parse_mode = false;
    static char parse_buffer[48];
    static int parse_index = 0;
    static char param;

    if (parse_mode) {
        if (c == '\n' || c == '\r') {
            parse_buffer[parse_index] = '\0';

            switch (param) {
            case 'P': handle_param_command(parse_buffer); break;
            case 'A': handle_axis_command(parse_buffer); break;
            case 'T': handle_test_command(parse_buffer); break;
            case 'I': handle_injection(parse_buffer); break;
            default: break;
            }

            parse_mode = false;
            parse_index = 0;
        } else if (parse_index < (int)sizeof(parse_buffer) - 1) {
            parse_buffer[parse_index++] = c;
        }
        return;
    }

    if (c == 'P' || c == 'A' || c == 'T' || c == 'I') {
        parse_mode = true;
        param = c;
        parse_index = 0;
        return;
    }

    switch (c) {
    case 'r':
        quat_identity(&s_q);
        printf("Reset orientation\n");
        break;
    case 'd':
        s_debug_stream = !s_debug_stream;
        printf(s_debug_stream ? "Debug stream ON - CSV format\n" : "Debug stream OFF\n");
        break;
    case 'p':
        print_config();
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
    printf("\n=== 6-DOF Configurable Kalman Filter ===\n");
    printf("Full axis configuration and testing\n");
    printf("Type 'h' for help\n");

    axis_defaults();
    quat_identity(&s_q);
    s_gx_bias = s_gy_bias = s_gz_bias = 0.0f;
    s_test_mode = TEST_OFF;
    s_debug_stream = false;

    const bool have_imu = qmi8658_present();
    if (!have_imu) {
        printf("No IMU detected: injection and test signals still work.\n");
    }

    uint64_t last_us = demo_micros();
    float filtered_gx = 0.0f, filtered_gy = 0.0f, filtered_gz = 0.0f;

    while (!demo_exit_requested()) {
        int c;
        while ((c = demo_read_char()) >= 0) {
            process_command((char)c);
        }

        vector3f_t acc = {0.0f, 0.0f, 1.0f};
        vector3f_t gyro = {0.0f, 0.0f, 0.0f};
        int16_t acc_raw[3] = {0, 0, 0};
        int16_t gyro_raw[3] = {0, 0, 0};

        if (have_imu) {
            qmi8658_read_raw(acc_raw, gyro_raw);
            demo_read_imu(&acc, &gyro);
        }

        const float raw_gyro[3] = {gyro.x * DEG_TO_RAD, gyro.y * DEG_TO_RAD,
                                   gyro.z * DEG_TO_RAD};
        const float raw_accel[3] = {acc.x, acc.y, acc.z};

        float configured_gyro[3];
        float configured_accel[3];
        apply_axis_config(raw_gyro, configured_gyro, s_axis.gyro_map, s_axis.gyro_sign,
                          s_axis.gyro_scale);
        apply_axis_config(raw_accel, configured_accel, s_axis.accel_map, s_axis.accel_sign,
                          s_axis.accel_scale);

        float gx = configured_gyro[0];
        float gy = configured_gyro[1];
        float gz = configured_gyro[2];
        float ax = configured_accel[0];
        float ay = configured_accel[1];
        float az = configured_accel[2];

        apply_test_signal(&gx, &gy, &gz, &ax, &ay, &az);

        filtered_gx = s_gyro_filter_alpha * filtered_gx + (1.0f - s_gyro_filter_alpha) * gx;
        filtered_gy = s_gyro_filter_alpha * filtered_gy + (1.0f - s_gyro_filter_alpha) * gy;
        filtered_gz = s_gyro_filter_alpha * filtered_gz + (1.0f - s_gyro_filter_alpha) * gz;

        const float dt = demo_delta_seconds(&last_us);

        quat_integrate(&s_q, filtered_gx - s_gx_bias, filtered_gy - s_gy_bias,
                       filtered_gz - s_gz_bias, dt);

        float roll, pitch, yaw;
        quat_to_euler(&s_q, &roll, &pitch, &yaw);

        if (s_debug_stream) {
            /* Same field order as the injection echo above. */
            printf("CSV,%lu,%.1f,%.1f,%.1f,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f\n",
                   (unsigned long)demo_millis(),
                   (double)gyro_raw[0], (double)gyro_raw[1], (double)gyro_raw[2],
                   (double)ax, (double)ay, (double)az,
                   (double)(roll * RAD_TO_DEG), (double)(pitch * RAD_TO_DEG),
                   (double)(yaw * RAD_TO_DEG));
        }

        demo_frame_begin(GFX_BLACK);

        vertex_t cube[8];
        for (int i = 0; i < 8; i++) {
            cube[i] = s_unit_cube[i];
            quat_rotate_vertex(&cube[i], &s_q);
        }
        demo_draw_cube_mono(cube, CUBE_DIST, CUBE_FOCAL, GFX_GREEN);

        gfx_printf(S(10), S(10), GFX_WHITE, 2, "P %+6.1f R %+6.1f Y %+6.1f",
                   (double)(pitch * RAD_TO_DEG), (double)(roll * RAD_TO_DEG),
                   (double)(yaw * RAD_TO_DEG));
        gfx_printf(S(10), S(22), GFX_GREY, 2, "gmap %d%d%d sign %+.0f%+.0f%+.0f",
                   s_axis.gyro_map[0], s_axis.gyro_map[1], s_axis.gyro_map[2],
                   (double)s_axis.gyro_sign[0], (double)s_axis.gyro_sign[1],
                   (double)s_axis.gyro_sign[2]);
        if (s_test_mode != TEST_OFF) {
            gfx_printf(S(10), S(34), GFX_YELLOW, 2, "test mode %d amp %.1f",
                       (int)s_test_mode, (double)(s_test_amplitude * RAD_TO_DEG));
        }
        if (!have_imu) {
            gfx_text(S(10), S(46), "no IMU: injection only", GFX_RED, 2);
        }
        demo_draw_exit_hint();

        demo_frame_end();
    }
    printf("\n");
}

const demo_t demo_kalman_6dof_config = {
    .name = "kalman_6dof_config",
    .summary = "Kalman filter with axis CLI and injection",
    .run = run,
};
