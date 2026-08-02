/**
 * 2.1" 480x480 round LCD (ST7701S, parallel RGB).
 *
 * Work-alike of lib/LCD/LCD_1in28.h from the RP2350 project. Same idea: hand
 * the driver a full RGB565 frame and it appears on the panel.
 *
 * The one API change the hardware forces: a 480x480x16bpp frame is 450 KB, far
 * too big for internal RAM, so the frame buffers are allocated in PSRAM by the
 * RGB panel driver and handed out by LCD_2IN1_GetBuffer(). Demos draw into that
 * pointer instead of into a static array of their own, then call
 * LCD_2IN1_Display() to put it on screen. With two buffers configured the call
 * flips between them, so drawing never lands on the frame being scanned out.
 */

#ifndef LCD_2IN1_H
#define LCD_2IN1_H

#include <stdint.h>

#include "esp_err.h"

#define LCD_2IN1_WIDTH  480
#define LCD_2IN1_HEIGHT 480

/* Kept for source compatibility with the 1.28" driver. */
#define HORIZONTAL 0
#define VERTICAL   1

typedef struct {
    uint16_t WIDTH;
    uint16_t HEIGHT;
    uint8_t SCAN_DIR;
} LCD_2IN1_ATTRIBUTES;

extern LCD_2IN1_ATTRIBUTES LCD_2IN1;

/** Reset the panel, push the ST7701S register sequence and start the RGB
 *  interface. Scan_dir is accepted for source compatibility; the panel is
 *  square so both directions give the same 480x480 geometry. */
esp_err_t LCD_2IN1_Init(uint8_t Scan_dir);

/** Always NULL: there is no frame buffer. Kept for source compatibility. */
uint16_t *LCD_2IN1_GetBuffer(void);

/** Hand the recorded frame to the panel and wait for it to be adopted, which
 *  is what makes the next frame safe to record. Callers need no delay of their
 *  own: the panel paces them. */
void LCD_2IN1_Present(void);

/** Source compatible spelling of LCD_2IN1_Present(). The argument is ignored,
 *  there being no frame buffer to hand over. */
void LCD_2IN1_Display(uint16_t *Image);

/** Longest and most recent time spent composing one bounce buffer, in
 *  microseconds. Reading the maximum resets it.
 *
 *  Going over the budget is caught by the composer, which abandons the rest of
 *  the buffer to make the deadline: see gfx_take_trimmed_rows() for why missing
 *  it is not survivable. That is a backstop, not a licence. Anything expensive
 *  should watch this figure and trim itself while it can still choose what to
 *  drop, rather than have whole rows taken off the bottom of every buffer. */
uint32_t LCD_2IN1_ComposeMaxUs(void);
uint32_t LCD_2IN1_ComposeLastUs(void);

/** Times the frame boundary failed to arrive within the backstop. Should stay
 *  at zero; anything else means composition is losing to the panel. */
uint32_t LCD_2IN1_Stalls(void);

/** How long composing one bounce buffer may take, in microseconds. Derived from
 *  the pixel clock: the time the other bounce buffer takes to drain. */
uint32_t LCD_2IN1_ComposeBudgetUs(void);

/** Fill every buffer with a colour and show it. */
void LCD_2IN1_Clear(uint16_t Color);

/* LCD_2IN1_DisplayWindows() and LCD_2IN1_DisplayPoint(), which the 1.28" driver
 * provides, are gone: both push pixels into a stored frame, and there is no
 * longer a frame to push them into. Draw with the gfx_* calls instead. */

/** Backlight, 0..100 percent (thin wrapper over DEV_SET_PWM). */
void LCD_2IN1_SetBacklight(uint8_t percent);

#endif /* LCD_2IN1_H */
