/**
 * Finger spun cube.
 *
 * New for this board, and the only cube here that never reads the IMU: drag it
 * to turn it, flick it to throw it, catch it to stop it. The board can sit flat
 * on the table throughout.
 *
 * It is a trackball. A drag turns the cube about an axis fixed to the screen
 * rather than one fixed to the cube, so the same gesture does the same thing
 * whatever the cube has already been through. That is the difference between a
 * left and a right multiply on the quaternion, and it is why this demo does not
 * use quat_integrate(): that one right multiplies, because gyro rates arrive in
 * the body frame.
 *
 * Speed matters as well as distance. A quick swipe turns the cube further than
 * a slow drag covering the same pixels, and the speed at the moment of release
 * becomes the spin it coasts away with.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "demo_common.h"

/* Wider angle than the sensor cubes: the perspective is what tells you which
 * way it is turning, and this one has no horizon or gravity marker to help. */
#define CUBE_DIST       200.0f
#define CUBE_FOCAL      180.0f

/* Dragging the full width of the panel turns the cube once around. */
#define FINGER_GAIN     (TWO_PI / (float)DISP_W)

/* Above this speed the drag starts winding on extra angle, up to the ceiling.
 * A careful drag stays near 1:1 so the cube can still be aimed. */
#define ACCEL_REF       1000.0f     /* pixels per second for +1x */
#define ACCEL_MAX       1.8f

/* A flick hands over a fraction of the finger's speed, the way a real ball
 * slips under a hand that is already lifting. Without it the spin ceiling is
 * reached at about 1000 px/s, which is an unremarkable swipe, and every flick
 * above that comes out identical. At this scale the ceiling needs a genuinely
 * hard throw, so the whole range of flick speeds stays distinguishable. */
#define FLING_SCALE     0.55f

#define VEL_TAU         0.06f       /* velocity smoothing, seconds */
#define VEL_STALE_US    120000ULL   /* movement older than this is not a flick */
#define JUMP_LIMIT      100         /* px in one frame: a replanted finger */
#define MAX_SPIN        19.0f       /* rad/s, three turns a second */
#define FRICTION        0.55f       /* per second */
#define SPIN_FLOOR      0.05f       /* below this it may as well be stopped */
#define AXIS_MIN        0.4f        /* rad/s before the axis line is drawn */

/** Turn the cube by a rotation vector expressed in screen axes: x right,
 *  y down, z away from the viewer. Left multiply, so the axis stays put while
 *  the cube moves under it. */
static void spin_world(quaternion_t *q, float rx, float ry, float rz)
{
    const float len2 = rx * rx + ry * ry + rz * rz;
    if (len2 < 1e-12f) {
        return;
    }

    const float theta = sqrtf(len2);
    const float half = 0.5f * theta;
    const float s = sinf(half) / theta;
    const quaternion_t delta = {cosf(half), rx * s, ry * s, rz * s};

    quaternion_t turned;
    quat_mul(&turned, &delta, q);
    *q = turned;
    quat_normalize(q);
}

static float speed_gain(float vx, float vy)
{
    const float speed = sqrtf(vx * vx + vy * vy);
    const float gain = 1.0f + speed / ACCEL_REF;
    return (gain > ACCEL_MAX) ? ACCEL_MAX : gain;
}

/* The axis a spin is about, drawn through the centre. Only the part of it that
 * lies in the screen plane has a direction to show, so dropping z shortens the
 * line as the axis tips toward the viewer, which is the foreshortening you
 * would see if the axis were a real rod. */
static void draw_spin_axis(float wx, float wy, float wz)
{
    const float mag = sqrtf(wx * wx + wy * wy + wz * wz);
    if (mag < AXIS_MIN) {
        return;
    }

    const int ex = (int)(wx / mag * 150.0f);
    const int ey = (int)(wy / mag * 150.0f);
    gfx_line(DISP_CX - ex, DISP_CY - ey, DISP_CX + ex, DISP_CY + ey, GFX_DGREY);
}

static void run(void)
{
    printf("Finger cube: drag to turn it, flick to throw it. No IMU.\n");

    if (!cst820_present()) {
        while (!demo_exit_requested()) {
            demo_show_message("NO TOUCH", "CST820 not detected", GFX_RED);
            demo_delay_ms(200);
        }
        return;
    }

    quaternion_t q;
    quat_identity(&q);

    /* A slow turn on entry, so it arrives reading as a solid rather than as a
     * flat outline that nobody has thought to touch yet. */
    float wx = 0.3f, wy = 0.9f, wz = 0.0f;

    bool held = false;
    int last_x = 0;
    int last_y = 0;
    float vx = 0.0f;
    float vy = 0.0f;
    uint64_t last_move_us = 0;
    uint64_t last_us = demo_micros();

    while (!demo_exit_requested()) {
        const float dt = demo_delta_seconds(&last_us);
        touch_state_t touch;
        const bool pressed = demo_touch(&touch);
        const uint64_t now = demo_micros();

        if (pressed && !held) {
            /* Caught. Whatever it was doing, it stops under the finger. */
            wx = wy = wz = 0.0f;
            vx = vy = 0.0f;
            last_x = (int)touch.x;
            last_y = (int)touch.y;
            last_move_us = now;
        } else if (pressed) {
            const int dx = (int)touch.x - last_x;
            const int dy = (int)touch.y - last_y;
            last_x = (int)touch.x;
            last_y = (int)touch.y;

            if (dx != 0 || dy != 0) {
                last_move_us = now;
            }

            /* A jump that large is a finger lifted and put down somewhere else,
             * not a drag. Take the new position, but do not turn the cube by the
             * gap between them. */
            if (abs(dx) < JUMP_LIMIT && abs(dy) < JUMP_LIMIT) {
                const float alpha = dt / (VEL_TAU + dt);
                vx += alpha * ((float)dx / dt - vx);
                vy += alpha * ((float)dy / dt - vy);

                /* Drag right and the near face goes right, which is a turn
                 * about the screen's up axis, and up is -y here. */
                const float gain = FINGER_GAIN * speed_gain(vx, vy);
                spin_world(&q, (float)dy * gain, -(float)dx * gain, 0.0f);
            }
        } else if (held) {
            /* Let go. The drag carries on as momentum, unless the finger had
             * already come to rest, in which case the cube was being parked and
             * throwing it now would be a surprise. */
            if (now - last_move_us < VEL_STALE_US) {
                /* Speed alone decides the throw. The drag's acceleration curve
                 * is deliberately not applied here: it would push all but the
                 * gentlest flicks straight into the ceiling. */
                const float gain = FINGER_GAIN * FLING_SCALE;
                wx = vy * gain;
                wy = -vx * gain;
                wz = 0.0f;

                float mag = sqrtf(wx * wx + wy * wy);
                if (mag > MAX_SPIN) {
                    const float trim = MAX_SPIN / mag;
                    wx *= trim;
                    wy *= trim;
                    mag = MAX_SPIN;
                }
                printf("flick %+6.0f %+6.0f px/s -> spin %4.0f deg/s\n",
                       (double)vx, (double)vy, (double)(mag * RAD_TO_DEG));
            }
            vx = vy = 0.0f;
        }
        held = pressed;

        if (!pressed) {
            spin_world(&q, wx * dt, wy * dt, wz * dt);

            float decay = 1.0f - FRICTION * dt;
            if (decay < 0.0f) {
                decay = 0.0f;
            }
            wx *= decay;
            wy *= decay;
            wz *= decay;
            if (sqrtf(wx * wx + wy * wy + wz * wz) < SPIN_FLOOR) {
                wx = wy = wz = 0.0f;
            }
        }

        demo_frame_begin(GFX_BLACK);

        vertex_t rotated[8];
        for (int i = 0; i < 8; i++) {
            rotated[i] = demo_cube_vertices[i];
            quat_rotate_vertex(&rotated[i], &q);
        }

        gfx_circle(DISP_CX, DISP_CY, 239, GFX_DGREY);
        draw_spin_axis(wx, wy, wz);
        demo_draw_cube(rotated, CUBE_DIST, CUBE_FOCAL);

        if (pressed) {
            gfx_circle(touch.x, touch.y, 20, GFX_GREY);
        }

        const float spin_dps = sqrtf(wx * wx + wy * wy + wz * wz) * RAD_TO_DEG;
        if (pressed) {
            gfx_text_centered(DISP_CX, DEMO_ROW_TOP(0), "HOLDING", GFX_CYAN, 2);
        } else if (spin_dps >= 1.0f) {
            char line[32];
            snprintf(line, sizeof(line), "spin %4.0f" GFX_DEG "/s", (double)spin_dps);
            gfx_text_centered(DISP_CX, DEMO_ROW_TOP(0), line, GFX_WHITE, 2);
        } else {
            gfx_text_centered(DISP_CX, DEMO_ROW_TOP(0), "DRAG TO SPIN", GFX_GREY, 2);
        }

        gfx_text_centered(DISP_CX, DEMO_ROW_BOTTOM(0), "flick to keep it going",
                          GFX_DGREY, 2);
        demo_draw_exit_hint();

        demo_frame_end();
    }
}

const demo_t demo_finger_cube = {
    .name = "finger_cube",
    .summary = "spin a cube with your finger",
    .run = run,
};
