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

/** The frame buffer that is safe to draw into right now. */
uint16_t *LCD_2IN1_GetBuffer(void);

/** Show a frame. Pass the pointer from LCD_2IN1_GetBuffer() to flip buffers;
 *  any other pointer is copied into the live frame instead.
 *
 *  Blocks until the panel has finished with the outgoing buffer, which is what
 *  makes the buffer returned by the next LCD_2IN1_GetBuffer() safe to draw
 *  into. Callers therefore need no delay of their own: the panel paces them. */
void LCD_2IN1_Display(uint16_t *Image);

/** Fill every buffer with a colour and show it. */
void LCD_2IN1_Clear(uint16_t Color);

/** Push a sub-rectangle of a full-size image. Coordinates are inclusive, which
 *  matches the 1.28" driver. */
void LCD_2IN1_DisplayWindows(uint16_t Xstart, uint16_t Ystart, uint16_t Xend, uint16_t Yend,
                             uint16_t *Image);

/** Set a single pixel straight on the panel. */
void LCD_2IN1_DisplayPoint(uint16_t X, uint16_t Y, uint16_t Color);

/** Backlight, 0..100 percent (thin wrapper over DEV_SET_PWM). */
void LCD_2IN1_SetBacklight(uint8_t percent);

#endif /* LCD_2IN1_H */
