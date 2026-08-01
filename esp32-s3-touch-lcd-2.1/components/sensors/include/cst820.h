/**
 * CST820 capacitive touch controller (single touch point plus gestures).
 *
 * The RP2350 board had no touch panel, so this driver has no counterpart in the
 * original tree. It is here because the demo launcher uses it as the on-device
 * menu, which replaces "flash a different .uf2" on the Pico.
 */

#ifndef CST820_H
#define CST820_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    TOUCH_GESTURE_NONE = 0x00,
    TOUCH_GESTURE_SWIPE_UP = 0x01,
    TOUCH_GESTURE_SWIPE_DOWN = 0x02,
    TOUCH_GESTURE_SWIPE_LEFT = 0x03,
    TOUCH_GESTURE_SWIPE_RIGHT = 0x04,
    TOUCH_GESTURE_SINGLE_CLICK = 0x05,
    TOUCH_GESTURE_DOUBLE_CLICK = 0x0B,
    TOUCH_GESTURE_LONG_PRESS = 0x0C,
} touch_gesture_t;

typedef struct {
    bool pressed;
    uint16_t x;
    uint16_t y;
    touch_gesture_t gesture;
} touch_state_t;

/** Reset the controller and check that it answers. */
bool cst820_init(void);

bool cst820_present(void);

/** Sample the panel. Returns true while a finger is down; *out is always
 *  filled in, with pressed = false when nothing is touching. */
bool cst820_read(touch_state_t *out);

const char *cst820_gesture_name(touch_gesture_t gesture);

#endif /* CST820_H */
