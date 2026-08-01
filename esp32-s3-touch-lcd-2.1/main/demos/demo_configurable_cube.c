/**
 * Configurable Madgwick cube with a serial CLI.
 *
 * Port of src/configurable_cube.c, the demo the RP2350 README leads with.
 * Everything about the axis mapping and the filter can be changed at runtime
 * over the USB serial console, which is how you work out the correct sensor
 * orientation for a new board.
 *
 * One fix carried in from the port: the RP2350 menu advertised six axis swap
 * keys that process_command() never implemented. They work here, with the
 * accelerometer on q/w/s and the gyroscope on a/z/x.
 */

#include <math.h>
#include <stdio.h>

#include "demo_common.h"

#define CUBE_DIST  200.0f
#define CUBE_FOCAL 120.0f

typedef struct {
    bool invert_ax, invert_ay, invert_az;
    bool invert_gx, invert_gy, invert_gz;
    int accel_map[3];
    int gyro_map[3];
    float beta;
    bool show_axes;
    bool show_data;
    bool show_cube;
    bool paused;
} config_t;

static config_t s_config;
static quaternion_t s_q;
static float s_last_acc[3];
static float s_last_gyro[3];

static void config_defaults(void)
{
    s_config = (config_t){
        .invert_ax = false, .invert_ay = false, .invert_az = false,
        .invert_gx = false, .invert_gy = true,  .invert_gz = false,
        .accel_map = {0, 1, 2},
        .gyro_map = {0, 1, 2},
        .beta = 0.1f,
        .show_axes = true,
        .show_data = true,
        .show_cube = true,
        .paused = false,
    };
}

static void swap_axes(int map[3], int a, int b)
{
    const int tmp = map[a];
    map[a] = map[b];
    map[b] = tmp;
}

static void read_configured_imu(float *ax, float *ay, float *az,
                                float *gx, float *gy, float *gz)
{
    vector3f_t acc;
    vector3f_t gyro;
    if (!demo_read_imu(&acc, &gyro)) {
        return;
    }

    float a[3] = {acc.x, acc.y, acc.z};
    float g[3] = {gyro.x * DEG_TO_RAD, gyro.y * DEG_TO_RAD, gyro.z * DEG_TO_RAD};

    if (s_config.invert_ax) {
        a[0] = -a[0];
    }
    if (s_config.invert_ay) {
        a[1] = -a[1];
    }
    if (s_config.invert_az) {
        a[2] = -a[2];
    }
    if (s_config.invert_gx) {
        g[0] = -g[0];
    }
    if (s_config.invert_gy) {
        g[1] = -g[1];
    }
    if (s_config.invert_gz) {
        g[2] = -g[2];
    }

    *ax = a[s_config.accel_map[0]];
    *ay = a[s_config.accel_map[1]];
    *az = a[s_config.accel_map[2]];
    *gx = g[s_config.gyro_map[0]];
    *gy = g[s_config.gyro_map[1]];
    *gz = g[s_config.gyro_map[2]];

    s_last_acc[0] = *ax;
    s_last_acc[1] = *ay;
    s_last_acc[2] = *az;
    s_last_gyro[0] = *gx;
    s_last_gyro[1] = *gy;
    s_last_gyro[2] = *gz;
}

static void print_menu(void)
{
    printf("\n\n=== Configurable Madgwick Cube Menu ===\n");
    printf("Axis controls:\n");
    printf("  1-6: toggle axis inversions (1=AX 2=AY 3=AZ 4=GX 5=GY 6=GZ)\n");
    printf("  q/w/s: swap accel axes (q=X<->Y w=X<->Z s=Y<->Z)\n");
    printf("  a/z/x: swap gyro axes  (a=X<->Y z=X<->Z x=Y<->Z)\n");
    printf("\nDisplay controls:\n");
    printf("  d: toggle sensor data display\n");
    printf("  c: toggle cube display\n");
    printf("  e: toggle axes display\n");
    printf("  p: pause/unpause\n");
    printf("\nFilter controls:\n");
    printf("  +/-: increase/decrease beta (current: %.3f)\n", (double)s_config.beta);
    printf("  r: reset orientation\n");
    printf("  b: recalibrate gyro bias\n");
    printf("\nOther:\n");
    printf("  v: view current sensor values\n");
    printf("  m: show this menu\n");
    printf("  i: show current axis configuration\n");
    printf("  ESC or ~: back to the launcher menu\n");
    printf("\nCurrent inversions: AX:%c AY:%c AZ:%c GX:%c GY:%c GZ:%c\n",
           s_config.invert_ax ? 'Y' : 'N', s_config.invert_ay ? 'Y' : 'N',
           s_config.invert_az ? 'Y' : 'N', s_config.invert_gx ? 'Y' : 'N',
           s_config.invert_gy ? 'Y' : 'N', s_config.invert_gz ? 'Y' : 'N');
}

static void calibrate_gyro(void)
{
    printf("Calibrating gyro... hold still\n");
    demo_show_message("CALIBRATING", "hold the board still", GFX_YELLOW);
    qmi8658_calibrate(200);
    const vector3f_t offset = qmi8658_gyro_offset();
    printf("Calibration complete: GX:%.2f GY:%.2f GZ:%.2f dps\n",
           (double)offset.x, (double)offset.y, (double)offset.z);
}

static void process_command(char cmd)
{
    switch (cmd) {
    case '1': s_config.invert_ax = !s_config.invert_ax;
        printf("AX inversion: %s\n", s_config.invert_ax ? "ON" : "OFF"); break;
    case '2': s_config.invert_ay = !s_config.invert_ay;
        printf("AY inversion: %s\n", s_config.invert_ay ? "ON" : "OFF"); break;
    case '3': s_config.invert_az = !s_config.invert_az;
        printf("AZ inversion: %s\n", s_config.invert_az ? "ON" : "OFF"); break;
    case '4': s_config.invert_gx = !s_config.invert_gx;
        printf("GX inversion: %s\n", s_config.invert_gx ? "ON" : "OFF"); break;
    case '5': s_config.invert_gy = !s_config.invert_gy;
        printf("GY inversion: %s\n", s_config.invert_gy ? "ON" : "OFF"); break;
    case '6': s_config.invert_gz = !s_config.invert_gz;
        printf("GZ inversion: %s\n", s_config.invert_gz ? "ON" : "OFF"); break;

    case 'q': swap_axes(s_config.accel_map, 0, 1); printf("Accel X<->Y swapped\n"); break;
    case 'w': swap_axes(s_config.accel_map, 0, 2); printf("Accel X<->Z swapped\n"); break;
    case 's': swap_axes(s_config.accel_map, 1, 2); printf("Accel Y<->Z swapped\n"); break;
    case 'a': swap_axes(s_config.gyro_map, 0, 1); printf("Gyro X<->Y swapped\n"); break;
    case 'z': swap_axes(s_config.gyro_map, 0, 2); printf("Gyro X<->Z swapped\n"); break;
    case 'x': swap_axes(s_config.gyro_map, 1, 2); printf("Gyro Y<->Z swapped\n"); break;

    case 'd': s_config.show_data = !s_config.show_data;
        printf("Data display: %s\n", s_config.show_data ? "ON" : "OFF"); break;
    case 'c': s_config.show_cube = !s_config.show_cube;
        printf("Cube display: %s\n", s_config.show_cube ? "ON" : "OFF"); break;
    case 'e': s_config.show_axes = !s_config.show_axes;
        printf("Axes display: %s\n", s_config.show_axes ? "ON" : "OFF"); break;
    case 'p': s_config.paused = !s_config.paused;
        printf("Display %s\n", s_config.paused ? "PAUSED" : "RUNNING"); break;

    case '+': s_config.beta += 0.01f;
        if (s_config.beta > 1.0f) {
            s_config.beta = 1.0f;
        }
        printf("Beta: %.3f\n", (double)s_config.beta); break;
    case '-': s_config.beta -= 0.01f;
        if (s_config.beta < 0.0f) {
            s_config.beta = 0.0f;
        }
        printf("Beta: %.3f\n", (double)s_config.beta); break;

    case 'r': quat_identity(&s_q); printf("Orientation reset\n"); break;
    case 'b': calibrate_gyro(); break;

    case 'v':
        printf("Sensors: AX:%.3f AY:%.3f AZ:%.3f GX:%.3f GY:%.3f GZ:%.3f\n",
               (double)s_last_acc[0], (double)s_last_acc[1], (double)s_last_acc[2],
               (double)s_last_gyro[0], (double)s_last_gyro[1], (double)s_last_gyro[2]);
        break;
    case 'm': print_menu(); break;
    case 'i':
        printf("Config: AX:%c AY:%c AZ:%c GX:%c GY:%c GZ:%c Beta:%.3f\n",
               s_config.invert_ax ? '-' : '+', s_config.invert_ay ? '-' : '+',
               s_config.invert_az ? '-' : '+', s_config.invert_gx ? '-' : '+',
               s_config.invert_gy ? '-' : '+', s_config.invert_gz ? '-' : '+',
               (double)s_config.beta);
        printf("Accel map: [%d,%d,%d]  Gyro map: [%d,%d,%d]\n",
               s_config.accel_map[0], s_config.accel_map[1], s_config.accel_map[2],
               s_config.gyro_map[0], s_config.gyro_map[1], s_config.gyro_map[2]);
        break;
    default:
        break;
    }
}

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

static void draw_sensor_data(void)
{
    static const uint16_t colors[3] = {GFX_RED, GFX_GREEN, GFX_BLUE};

    /* Accelerometer bars along the top, gyro bars along the bottom, same as
     * the original, plus the numbers the original had no font to show. */
    for (int i = 0; i < 3; i++) {
        const int y = S(10) + i * S(6);
        const int len = (int)(fabsf(s_last_acc[i]) * S(30));
        gfx_fill_rect(S(10), y, (s_last_acc[i] >= 0) ? len : -len, 4, colors[i]);
    }
    for (int i = 0; i < 3; i++) {
        const int y = S(220) + i * S(6);
        const int len = (int)(fabsf(s_last_gyro[i]) * S(20));
        gfx_fill_rect(S(10), y, (s_last_gyro[i] >= 0) ? len : -len, 4, colors[i]);
    }

    gfx_printf(S(120), S(8), GFX_WHITE, 2, "a %+.2f %+.2f %+.2f",
               (double)s_last_acc[0], (double)s_last_acc[1], (double)s_last_acc[2]);
    gfx_printf(S(120), S(20), GFX_WHITE, 2, "g %+.2f %+.2f %+.2f",
               (double)s_last_gyro[0], (double)s_last_gyro[1], (double)s_last_gyro[2]);
}

static void run(void)
{
    config_defaults();
    quat_identity(&s_q);

    printf("Configurable Madgwick cube. Press 'm' for the menu.\n");

    if (!qmi8658_present()) {
        while (!demo_exit_requested()) {
            demo_show_message("NO IMU", "QMI8658 not detected", GFX_RED);
            demo_delay_ms(200);
        }
        return;
    }

    calibrate_gyro();
    print_menu();

    uint64_t last_us = demo_micros();

    while (!demo_exit_requested()) {
        int c;
        while ((c = demo_read_char()) >= 0) {
            process_command((char)c);
        }

        if (s_config.paused) {
            demo_delay_ms(20);
            continue;
        }

        float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
        read_configured_imu(&ax, &ay, &az, &gx, &gy, &gz);

        const float dt = demo_delta_seconds(&last_us);
        madgwick_update(&s_q, s_config.beta, gx, gy, gz, ax, ay, az, dt);

        demo_frame_begin(GFX_BLACK);

        if (s_config.show_cube) {
            vertex_t rotated[8];
            for (int i = 0; i < 8; i++) {
                rotated[i] = demo_cube_vertices[i];
                quat_rotate_vertex(&rotated[i], &s_q);
            }
            demo_draw_cube(rotated, CUBE_DIST, CUBE_FOCAL);
        }
        if (s_config.show_axes) {
            draw_axes(&s_q);
        }
        if (s_config.show_data) {
            draw_sensor_data();
        }

        gfx_printf(S(10), S(196), GFX_DGREY, 2, "beta %.2f  inv %c%c%c%c%c%c",
                   (double)s_config.beta,
                   s_config.invert_ax ? 'X' : '-', s_config.invert_ay ? 'Y' : '-',
                   s_config.invert_az ? 'Z' : '-', s_config.invert_gx ? 'X' : '-',
                   s_config.invert_gy ? 'Y' : '-', s_config.invert_gz ? 'Z' : '-');
        demo_draw_exit_hint();

        demo_frame_end();
        demo_delay_ms(10);
    }
    printf("\n");
}

const demo_t demo_configurable_cube = {
    .name = "configurable_cube",
    .summary = "interactive cube with serial CLI",
    .run = run,
};
