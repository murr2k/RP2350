/**
 * Buffered rotating cube.
 *
 * Port of src/buffered_cube.c. No sensors involved: it spins a wireframe cube
 * at fixed rates to show the frame buffer path end to end. The colour flash at
 * the start is from the original and doubles as a panel check.
 */

#include <stdio.h>

#include "demo_common.h"

/* Original cube is +/-40 units, projected with a focal length of 100 from
 * 150 units away. */
#define CUBE_HALF   40.0f
#define CUBE_DIST   150.0f
#define CUBE_FOCAL  100.0f

static void run(void)
{
    printf("Buffered rotating cube. Nothing but graphics, no IMU needed.\n");

    const uint16_t flash[3] = {GFX_RED, GFX_GREEN, GFX_BLUE};
    for (int i = 0; i < 3 && !demo_exit_requested(); i++) {
        demo_frame_begin(flash[i]);
        demo_frame_end();
        demo_delay_ms(600);
    }

    float angle_x = 0.0f;
    float angle_y = 0.0f;
    float angle_z = 0.0f;
    uint32_t frame = 0;

    while (!demo_exit_requested()) {
        demo_frame_begin(GFX_BLACK);

        vertex_t rotated[8] = {
            {-CUBE_HALF, -CUBE_HALF, -CUBE_HALF}, { CUBE_HALF, -CUBE_HALF, -CUBE_HALF},
            { CUBE_HALF,  CUBE_HALF, -CUBE_HALF}, {-CUBE_HALF,  CUBE_HALF, -CUBE_HALF},
            {-CUBE_HALF, -CUBE_HALF,  CUBE_HALF}, { CUBE_HALF, -CUBE_HALF,  CUBE_HALF},
            { CUBE_HALF,  CUBE_HALF,  CUBE_HALF}, {-CUBE_HALF,  CUBE_HALF,  CUBE_HALF},
        };
        for (int i = 0; i < 8; i++) {
            rotate_x(&rotated[i], angle_x);
            rotate_y(&rotated[i], angle_y);
            rotate_z(&rotated[i], angle_z);
        }

        gfx_circle(DISP_CX, DISP_CY, S(115), GFX_CYAN);
        demo_draw_cube(rotated, CUBE_DIST, CUBE_FOCAL);

        /* Frame counter, drawn as dots in the original because it had no font.
         * Both are here now: the dots and the number. */
        for (uint32_t i = 0; i < (frame % 10); i++) {
            gfx_fill_rect(S(10) + (int)i * S(3), S(10), 3, 3, GFX_YELLOW);
        }
        gfx_printf(S(10), S(20), GFX_YELLOW, 2, "frame %lu", (unsigned long)frame);
        gfx_text_centered(DISP_CX, DISP_H - 70, "buffered_cube", GFX_DGREY, 2);
        demo_draw_exit_hint();

        demo_frame_end();

        angle_x += 0.03f;
        angle_y += 0.05f;
        angle_z += 0.01f;
        if (angle_x > TWO_PI) {
            angle_x -= TWO_PI;
        }
        if (angle_y > TWO_PI) {
            angle_y -= TWO_PI;
        }
        if (angle_z > TWO_PI) {
            angle_z -= TWO_PI;
        }

        frame++;
        if ((frame % 20) == 0) {
            printf("\rframe %lu", (unsigned long)frame);
        }

        /* No delay: demo_frame_end() blocks until the panel releases a buffer. */
    }
    printf("\n");
}

const demo_t demo_buffered_cube = {
    .name = "buffered_cube",
    .summary = "spinning wireframe cube, no sensors",
    .run = run,
};
