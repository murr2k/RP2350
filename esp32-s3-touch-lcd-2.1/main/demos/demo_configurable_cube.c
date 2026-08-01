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

/* Runs on the sensor core at 250 Hz. The configuration it reads is written by
 * the CLI on the render core, so it is copied under the lock rather than used
 * field by field, which would let a half applied change through. */
static void filter_step(const imu_sample_t *sample)
{
    config_t cfg;
    quaternion_t q;

    demo_lock();
    cfg = s_config;
    q = s_q;
    demo_unlock();

    if (cfg.paused) {
        return;
    }

    float a[3] = {sample->acc.x, sample->acc.y, sample->acc.z};
    float g[3] = {sample->gyro.x * DEG_TO_RAD, sample->gyro.y * DEG_TO_RAD,
                  sample->gyro.z * DEG_TO_RAD};

    if (cfg.invert_ax) {
        a[0] = -a[0];
    }
    if (cfg.invert_ay) {
        a[1] = -a[1];
    }
    if (cfg.invert_az) {
        a[2] = -a[2];
    }
    if (cfg.invert_gx) {
        g[0] = -g[0];
    }
    if (cfg.invert_gy) {
        g[1] = -g[1];
    }
    if (cfg.invert_gz) {
        g[2] = -g[2];
    }

    const float ax = a[cfg.accel_map[0]];
    const float ay = a[cfg.accel_map[1]];
    const float az = a[cfg.accel_map[2]];
    const float gx = g[cfg.gyro_map[0]];
    const float gy = g[cfg.gyro_map[1]];
    const float gz = g[cfg.gyro_map[2]];

    madgwick_update(&q, cfg.beta, gx, gy, gz, ax, ay, az, sample->dt);

    demo_lock();
    s_q = q;
    s_last_acc[0] = ax;
    s_last_acc[1] = ay;
    s_last_acc[2] = az;
    s_last_gyro[0] = gx;
    s_last_gyro[1] = gy;
    s_last_gyro[2] = gz;
    demo_unlock();
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

    case 'r':
        demo_lock();
        quat_identity(&s_q);
        demo_unlock();
        printf("Orientation reset\n");
        break;
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

    /* Accelerometer near the top, gyro near the bottom, same as the original.
     * The bars now grow out from the centre column rather than from the left
     * edge, which keeps them inside the circle and makes the sign readable at
     * a glance. */
    char line[48];

    snprintf(line, sizeof(line), "a %+.2f %+.2f %+.2f",
             (double)s_last_acc[0], (double)s_last_acc[1], (double)s_last_acc[2]);
    gfx_text_centered(DISP_CX, DEMO_ROW_TOP(0), line, GFX_WHITE, 2);
    for (int i = 0; i < 3; i++) {
        gfx_fill_rect(DISP_CX, DEMO_ROW_TOP(1) + i * 6,
                      (int)(s_last_acc[i] * S(30)), 4, colors[i]);
    }

    snprintf(line, sizeof(line), "g %+.2f %+.2f %+.2f",
             (double)s_last_gyro[0], (double)s_last_gyro[1], (double)s_last_gyro[2]);
    gfx_text_centered(DISP_CX, DEMO_ROW_BOTTOM(1), line, GFX_WHITE, 2);
    for (int i = 0; i < 3; i++) {
        gfx_fill_rect(DISP_CX, DEMO_ROW_BOTTOM(0) + i * 6,
                      (int)(s_last_gyro[i] * S(20)), 4, colors[i]);
    }
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
    demo_set_filter(filter_step);

    while (!demo_exit_requested()) {
        int c;
        while ((c = demo_read_char()) >= 0) {
            process_command((char)c);
        }

        if (s_config.paused) {
            demo_delay_ms(20);
            continue;
        }

        quaternion_t q;
        demo_lock();
        q = s_q;
        demo_unlock();

        demo_frame_begin(GFX_BLACK);

        if (s_config.show_cube) {
            vertex_t rotated[8];
            for (int i = 0; i < 8; i++) {
                rotated[i] = demo_cube_vertices[i];
                quat_rotate_vertex(&rotated[i], &q);
            }
            demo_draw_cube(rotated, CUBE_DIST, CUBE_FOCAL);
        }
        if (s_config.show_axes) {
            draw_axes(&q);
        }
        if (s_config.show_data) {
            draw_sensor_data();
        }

        char status[48];
        snprintf(status, sizeof(status), "beta %.2f  inv %c%c%c%c%c%c",
                 (double)s_config.beta,
                 s_config.invert_ax ? 'X' : '-', s_config.invert_ay ? 'Y' : '-',
                 s_config.invert_az ? 'Z' : '-', s_config.invert_gx ? 'X' : '-',
                 s_config.invert_gy ? 'Y' : '-', s_config.invert_gz ? 'Z' : '-');
        gfx_text_centered(DISP_CX, DEMO_ROW_BOTTOM(2), status, GFX_DGREY, 2);
        demo_draw_exit_hint();

        demo_frame_end();
    }
    demo_set_filter(NULL);
    printf("\n");
}

const demo_t demo_configurable_cube = {
    .name = "configurable_cube",
    .summary = "interactive cube with serial CLI",
    .run = run,
};
