/**
 * Intuitive gravity cube.
 *
 * Port of src/intuitive_cube.c. Accelerometer only: the cube keeps its place in
 * the world while the board turns around it, through any attitude, with a
 * crosshair over the raised side.
 *
 * The original took asin() of two accelerometer components and applied them as
 * Euler angles, one axis after the other. That cannot do the job and the code
 * admitted it, clamping both to 45 degrees. Three separate reasons:
 *
 *   asin() reaches only a right angle, and gives the same answer either side of
 *   it, so past vertical the board reads as though it were coming back.
 *
 *   Two angles applied in order lose a degree of freedom when the second lines
 *   the first up with the third, which for a board is standing it on edge. That
 *   is gimbal lock, and no choice of angles avoids it.
 *
 *   Naming an angle about a fixed axis fixes the axis, and turning the board
 *   over is what makes that axis the wrong one to have named.
 *
 * So no angle is named here. The accelerometer says which way is up in board
 * axes, and that pins down two of the three degrees of freedom on its own. Each
 * sample turns the cube's own idea of up a fraction of the way onto the measured
 * one, along the shortest arc between them, and composes that onto the
 * quaternion it already had. Small turns, multiplied: nothing to lock, no sign
 * to flip, and every attitude reachable including upside down.
 *
 * The third degree of freedom, heading about the gravity vector, an
 * accelerometer cannot see: gravity is unchanged by turning the board on the
 * table. It is carried over from the previous sample rather than invented, so
 * the cube yaws with the board and holds still otherwise. Recovering it needs
 * the gyroscope, which is what madgwick_cube and the Kalman demos are for.
 *
 * Two things about the direction, both counterintuitive, both wrong here once:
 *
 * The axis: the tilt of one edge turns the cube about the axis at right angles
 * to the one that edge reads on, because an edge levers the board about the axis
 * running across from it. Pairing them by matching name, x to x and y to y, is
 * what made lifting the right hand edge tumble the cube forwards.
 *
 * The direction: against the tilt, not with it. The screen is a window onto the
 * cube rather than a tray carrying it, so lifting an edge moves the window and
 * leaves the cube where it was. Both fall out of the arc above without being
 * asked for, which is the other argument for doing it this way.
 */

#include <math.h>
#include <stdio.h>

#include "demo_common.h"

#define CUBE_DIST   200.0f
#define CUBE_FOCAL  120.0f

/* How long the cube takes to settle onto a new attitude. The original low pass
 * had a time constant near 0.3 s and this is the same idea, applied to the arc
 * rather than to the components, so the easing no longer depends on which way
 * up the board is. */
#define ALIGN_TAU   0.25f

/* Below this the reading is a shake or a drop, and holds no information about
 * which way up the world is. Better to coast than to follow it. */
#define ALIGN_MIN_G 0.3f

/* Written on the sensor core, read on the render core. */
static quaternion_t s_q;
static vertex_t s_up;           /* measured up, in the projection's axes */
static bool s_settled;

static void filter_step(const imu_sample_t *sample)
{
    /* The IMU frame puts +Y toward the top of the screen and +Z out of it. The
     * projection puts +y down the screen and +z away. Same handedness, half a
     * turn about x between them. */
    vertex_t up = {sample->acc.x, -sample->acc.y, -sample->acc.z};

    const float mag = sqrtf(up.x * up.x + up.y * up.y + up.z * up.z);
    if (mag < ALIGN_MIN_G) {
        return;
    }
    up.x /= mag;
    up.y /= mag;
    up.z /= mag;

    quaternion_t q;
    bool settled;
    demo_lock();
    q = s_q;
    settled = s_settled;
    demo_unlock();

    /* Where the cube is currently holding its up. */
    vertex_t held = {0.0f, 0.0f, -1.0f};
    quat_rotate_vertex(&held, &q);

    /* Part of the way from there onto the measurement. The first sample goes
     * the whole way, so entering the demo with the board already tilted does
     * not start with the cube swinging up from flat. */
    const float fraction = settled ? (sample->dt / (ALIGN_TAU + sample->dt)) : 1.0f;
    quaternion_t delta;
    quat_align(&delta, &held, &up, fraction);

    /* Left multiply: the correction is an arc between two directions in the
     * world, not a turn about one of the cube's own axes. */
    quaternion_t turned;
    quat_mul(&turned, &delta, &q);
    quat_normalize(&turned);

    demo_lock();
    s_q = turned;
    s_up = up;
    s_settled = true;
    demo_unlock();
}

/* The marker sits over whichever part of the board is highest, so it moves the
 * way the board is lifted: a spirit level, not a ball. Accelerometer readings
 * are the up direction in board axes, so they can be used as they are, once the
 * screen's downward y is accounted for. */
static void draw_horizon(float up_right, float up_top)
{
    gfx_line(S(20), DISP_CY, S(220), DISP_CY, GFX_DGREEN);
    gfx_line(DISP_CX, S(20), DISP_CX, S(220), GFX_DGREEN);

    int tx = DISP_CX + (int)(up_right * 100.0f * DISP_SCALE);
    int ty = DISP_CY - (int)(up_top * 100.0f * DISP_SCALE);

    if (tx < S(20)) {
        tx = S(20);
    }
    if (tx > S(220)) {
        tx = S(220);
    }
    if (ty < S(20)) {
        ty = S(20);
    }
    if (ty > S(220)) {
        ty = S(220);
    }

    gfx_line(tx - S(10), ty, tx + S(10), ty, GFX_YELLOW);
    gfx_line(tx, ty - S(10), tx, ty + S(10), GFX_YELLOW);
    gfx_circle(tx, ty, 6, GFX_YELLOW);
}

static void run(void)
{
    printf("Intuitive gravity cube: the cube follows the board's tilt.\n");

    if (!qmi8658_present()) {
        while (!demo_exit_requested()) {
            demo_show_message("NO IMU", "QMI8658 not detected", GFX_RED);
            demo_delay_ms(200);
        }
        return;
    }

    quat_identity(&s_q);
    s_up = (vertex_t){0.0f, 0.0f, -1.0f};
    s_settled = false;
    demo_set_filter(filter_step);

    while (!demo_exit_requested()) {
        quaternion_t q;
        vertex_t up;
        demo_lock();
        q = s_q;
        up = s_up;
        demo_unlock();

        demo_frame_begin(GFX_BLACK);

        vertex_t rotated[8];
        for (int i = 0; i < 8; i++) {
            rotated[i] = demo_cube_vertices[i];
            quat_rotate_vertex(&rotated[i], &q);
        }

        /* Screen up is -y, and flat holds up at -z, so the tilt away from level
         * is the angle between the two. It runs to a full half turn now rather
         * than stopping at a right angle. */
        draw_horizon(up.x, -up.y);
        demo_draw_cube(rotated, CUBE_DIST, CUBE_FOCAL);

        float cos_tilt = -up.z;
        if (cos_tilt > 1.0f) {
            cos_tilt = 1.0f;
        }
        if (cos_tilt < -1.0f) {
            cos_tilt = -1.0f;
        }
        const float tilt = acosf(cos_tilt) * RAD_TO_DEG;

        char line[48];
        snprintf(line, sizeof(line), "tilt %5.1f" GFX_DEG, (double)tilt);
        gfx_text_centered(DISP_CX, DEMO_ROW_TOP(0), line, GFX_WHITE, 2);

        /* Back in board axes, which is how the other demos and the board
         * documentation talk about it. */
        snprintf(line, sizeof(line), "up %+.2f %+.2f %+.2f",
                 (double)up.x, (double)(-up.y), (double)(-up.z));
        gfx_text_centered(DISP_CX, DEMO_ROW_BOTTOM(0), line, GFX_GREY, 2);
        demo_draw_exit_hint();

        demo_frame_end();

        printf("\rTilt: %6.1f | Up: %+.2f %+.2f %+.2f      ",
               (double)tilt, (double)up.x, (double)(-up.y), (double)(-up.z));
    }
    demo_set_filter(NULL);
    printf("\n");
}

const demo_t demo_intuitive_cube = {
    .name = "intuitive_cube",
    .summary = "accelerometer tilt cube",
    .run = run,
};
