/**
 * Shared scaffolding for the demos.
 *
 * On the RP2350 each demo was its own firmware image and every one of them
 * carried a private copy of the frame buffer, the Bresenham line routine, the
 * cube vertices and the quaternion helpers. ESP-IDF builds a single image, so
 * the demos are functions here and the duplicated pieces live in one place.
 *
 * Geometry: the demos were written against a 240x240 screen with hardcoded
 * centres of 120. This panel is 480x480, so coordinates keep their original
 * values and get scaled by DISP_SCALE on the way to the frame buffer. The
 * pictures come out identical, just at twice the resolution.
 */

#ifndef DEMO_COMMON_H
#define DEMO_COMMON_H

#include <stdbool.h>
#include <stdint.h>

#include "cst820.h"
#include "dev_config.h"
#include "gfx.h"
#include "lcd_2in1.h"
#include "qmi8658.h"

/* --- screen --------------------------------------------------------------- */

#define DISP_W      LCD_2IN1_WIDTH
#define DISP_H      LCD_2IN1_HEIGHT
#define DISP_CX     (DISP_W / 2)
#define DISP_CY     (DISP_H / 2)
#define DISP_SCALE  2.0f                        /* 480 / 240 */
#define S(v)        ((int)((v) * DISP_SCALE))   /* 1.28" coordinate -> this panel */

/* --- maths ---------------------------------------------------------------- */

#define PI          3.14159265359f
#define TWO_PI      6.28318530718f
#define DEG_TO_RAD  (PI / 180.0f)
#define RAD_TO_DEG  (180.0f / PI)

typedef struct {
    float x, y, z;
} vertex_t;

typedef struct {
    float w, x, y, z;
} quaternion_t;

extern const vertex_t demo_cube_vertices[8];    /* +/-50 units, as in the originals */
extern const int demo_cube_edges[12][2];

float invSqrt(float x);

void quat_identity(quaternion_t *q);
void quat_normalize(quaternion_t *q);
void quat_integrate(quaternion_t *q, float gx, float gy, float gz, float dt);
void quat_rotate_vertex(vertex_t *v, const quaternion_t *q);
void quat_to_euler(const quaternion_t *q, float *roll, float *pitch, float *yaw);
void quat_to_gravity(const quaternion_t *q, float *gx, float *gy, float *gz);

void madgwick_update(quaternion_t *q, float beta, float gx, float gy, float gz,
                     float ax, float ay, float az, float dt);

void rotate_x(vertex_t *v, float angle);
void rotate_y(vertex_t *v, float angle);
void rotate_z(vertex_t *v, float angle);

/** Perspective projection matching the RP2350 demos: the camera sits
 *  `distance` units away with the given focal length, both in the original
 *  240x240 coordinate system, scaled up to this panel. */
void demo_project(vertex_t v, float distance, float focal, int *x, int *y);

/** Draw the standard 12 edge wireframe with the original per-face colours. */
void demo_draw_cube(const vertex_t *vertices, float distance, float focal);
void demo_draw_cube_mono(const vertex_t *vertices, float distance, float focal, uint16_t color);

/* --- IMU ------------------------------------------------------------------ */

/** Read the sensor with the board level axis map from board_config.h applied.
 *  Acceleration comes back in g, angular rate in degrees per second.
 *  Returns false if the IMU is missing or the bus errored. */
bool demo_read_imu(vector3f_t *acc, vector3f_t *gyro);

/* --- launcher services ---------------------------------------------------- */

typedef struct {
    const char *name;       /**< matches the RP2350 target name */
    const char *summary;
    void (*run)(void);      /**< returns when demo_exit_requested() goes true */
} demo_t;

/** True once the user asked to go back to the menu: ESC or '~' on the serial
 *  console, or a finger held near the top of the screen. */
bool demo_exit_requested(void);

/** Next character from the serial console, or -1 if nothing is waiting.
 *  Work-alike of the Pico's getchar_timeout_us(0). */
int demo_read_char(void);

/** Discard anything already typed. */
void demo_flush_input(void);

/** Clear the exit flag. The launcher calls this before starting a demo. */
void demo_clear_exit(void);

void demo_delay_ms(uint32_t ms);
uint32_t demo_millis(void);
uint64_t demo_micros(void);

/** Seconds since the previous call, clamped to a sane range. Pass the same
 *  state variable each time. */
float demo_delta_seconds(uint64_t *last_us);

/** Grab the drawable frame buffer, bind gfx to it and clear it. */
uint16_t *demo_frame_begin(uint16_t clear_color);

/** Put the bound frame on the panel and wait for the handover to complete.
 *  This is the render loop's pacing: demos need no delay of their own. */
void demo_frame_end(void);

/** Latest touch sample, polled by the launcher every frame. */
bool demo_touch(touch_state_t *out);

/** Small hint strip drawn at the bottom of a demo, mirroring the exit rule. */
void demo_draw_exit_hint(void);

/** Full screen message, used while calibrating and on fatal sensor errors. */
void demo_show_message(const char *title, const char *detail, uint16_t color);

/* The demos, in menu order. */
extern const demo_t demo_display_test;
extern const demo_t demo_buffered_cube;
extern const demo_t demo_intuitive_cube;
extern const demo_t demo_gravity_locked_cube;
extern const demo_t demo_madgwick_cube;
extern const demo_t demo_configurable_cube;
extern const demo_t demo_rotation_test;
extern const demo_t demo_axis_test;
extern const demo_t demo_kalman_6dof;
extern const demo_t demo_kalman_6dof_config;
extern const demo_t demo_touch_test;
extern const demo_t demo_diagnostic;

#endif /* DEMO_COMMON_H */
