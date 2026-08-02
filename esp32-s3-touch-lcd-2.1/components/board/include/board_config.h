/**
 * Board pin map: Waveshare ESP32-S3-Touch-LCD-2.1
 *
 * Work-alike of include/board_config.h + lib/Config/DEV_Config.h from the
 * RP2350-LCD-1.28 project. Everything the port needs to know about the
 * hardware lives here.
 *
 * Board summary
 *   MCU     ESP32-S3R8, dual Xtensa LX7 @ 240 MHz, 8 MB octal PSRAM
 *   LCD     2.1" round IPS, 480x480, ST7701S, 16-bit parallel RGB + 3-wire SPI
 *   Touch   CST820 capacitive controller on I2C (single touch point)
 *   IMU     QMI8658 6-axis accelerometer + gyroscope on I2C (same part as the
 *           RP2350 board, so the sensor code carries over unchanged)
 *   RTC     PCF85063 on I2C
 *   Expander TCA9554PWR on I2C, drives LCD reset, touch reset and LCD chip select
 *   Other   microSD (SDMMC 1-bit), battery divider on ADC1_CH3
 */

#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

/* ---------------------------------------------------------------- display -- */

#define BOARD_LCD_WIDTH             480
#define BOARD_LCD_HEIGHT            480

/* 3-wire SPI used only for the ST7701S power-on register sequence. */
#define BOARD_LCD_SPI_HOST          SPI2_HOST
#define BOARD_LCD_SPI_CLK_GPIO      2
#define BOARD_LCD_SPI_MOSI_GPIO     1
#define BOARD_LCD_SPI_HZ            (10 * 1000 * 1000)

/* Parallel RGB interface (RGB565: B0..B4, G0..G5, R0..R4). */
#define BOARD_LCD_RGB_HSYNC_GPIO    38
#define BOARD_LCD_RGB_VSYNC_GPIO    39
#define BOARD_LCD_RGB_DE_GPIO       40
#define BOARD_LCD_RGB_PCLK_GPIO     41
#define BOARD_LCD_RGB_DISP_GPIO     (-1)

#define BOARD_LCD_RGB_DATA_GPIOS { \
     5, /* D0  B0 */               \
    45, /* D1  B1 */               \
    48, /* D2  B2 */               \
    47, /* D3  B3 */               \
    21, /* D4  B4 */               \
    14, /* D5  G0 */               \
    13, /* D6  G1 */               \
    12, /* D7  G2 */               \
    11, /* D8  G3 */               \
    10, /* D9  G4 */               \
     9, /* D10 G5 */               \
    46, /* D11 R0 */               \
     3, /* D12 R1 */               \
     8, /* D13 R2 */               \
    18, /* D14 R3 */               \
    17, /* D15 R4 */               \
}

/* Panel timing, porches from the Waveshare ST7701S sample for this board.
 *
 * The pixel clock is below Waveshare's 16 MHz on purpose. Nothing here needs
 * 58 fps, and since the display is composed on demand rather than streamed from
 * a frame buffer, the pixel clock is what sets how long the interrupt has to
 * produce each row. Slower clock, more time per row, more margin against the
 * composer being late. 11 MHz gives about 40 fps and 50 us per row where 16 MHz
 * gave 58 fps and 34 us. */
#define BOARD_LCD_PCLK_HZ           (11 * 1000 * 1000)
#define BOARD_LCD_HSYNC_PULSE_WIDTH 8
#define BOARD_LCD_HSYNC_BACK_PORCH  10
#define BOARD_LCD_HSYNC_FRONT_PORCH 50
#define BOARD_LCD_VSYNC_PULSE_WIDTH 3
#define BOARD_LCD_VSYNC_BACK_PORCH  8
#define BOARD_LCD_VSYNC_FRONT_PORCH 8

/* Everything timing related is derived from those, so changing the clock cannot
 * leave a stale constant behind. */
#define BOARD_LCD_H_TOTAL (BOARD_LCD_HSYNC_PULSE_WIDTH + BOARD_LCD_HSYNC_BACK_PORCH + \
                           BOARD_LCD_WIDTH + BOARD_LCD_HSYNC_FRONT_PORCH)
#define BOARD_LCD_V_TOTAL (BOARD_LCD_VSYNC_PULSE_WIDTH + BOARD_LCD_VSYNC_BACK_PORCH + \
                           BOARD_LCD_HEIGHT + BOARD_LCD_VSYNC_FRONT_PORCH)
/* Scaled so the intermediate stays inside 32 bits: H_TOTAL * 1e9 does not. */
#define BOARD_LCD_LINE_NS ((BOARD_LCD_H_TOTAL * 1000u) / (BOARD_LCD_PCLK_HZ / 1000000u))
#define BOARD_LCD_REFRESH_HZ (BOARD_LCD_PCLK_HZ / (BOARD_LCD_H_TOTAL * BOARD_LCD_V_TOTAL))

/* Backlight is a plain GPIO driven by LEDC (the Pico used a PWM slice). */
#define BOARD_LCD_BL_GPIO           6
#define BOARD_LCD_BL_FREQ_HZ        20000
#define BOARD_LCD_BL_RES_BITS       10      /* duty range 0..1023 */

/* ------------------------------------------------------------------- I2C --- */

/* One shared bus: touch, IMU, RTC and the IO expander all hang off it. */
#define BOARD_I2C_PORT              I2C_NUM_0
#define BOARD_I2C_SDA_GPIO          15
#define BOARD_I2C_SCL_GPIO          7
#define BOARD_I2C_HZ                400000

/* ------------------------------------------------------- IO expander ------ */

#define BOARD_TCA9554_ADDR          0x20
/* Expander pins are numbered 1..8 to match the Waveshare EXIO naming. */
#define BOARD_EXIO_LCD_RST          1
#define BOARD_EXIO_TOUCH_RST        2
#define BOARD_EXIO_LCD_CS           3
/* The buzzer is active high and sounds continuously while this pin is driven
 * high, so it has to be parked low the moment the expander becomes an output. */
#define BOARD_EXIO_BUZZER           8

/* Output register value at startup: everything the port drives is active low
 * and therefore idles high, except the buzzer. */
#define BOARD_EXIO_IDLE_STATE       0x7F

/* ------------------------------------------------------------------- IMU --- */

#define BOARD_IMU_ADDR              0x6B    /* QMI8658, SA0 high */
#define BOARD_IMU_ADDR_ALT          0x6A

/* ----------------------------------------------------------------- touch --- */

#define BOARD_TOUCH_ADDR            0x15    /* CST820 */
#define BOARD_TOUCH_INT_GPIO        16

/* ------------------------------------------------------------------- RTC --- */

#define BOARD_RTC_ADDR              0x51    /* PCF85063 */

/* --------------------------------------------------------------- battery --- */

/* Divider feeding ADC1_CH3; multiply the measured millivolts by 3. */
#define BOARD_BAT_ADC_GPIO          4
#define BOARD_BAT_ADC_CHANNEL       ADC_CHANNEL_3
#define BOARD_BAT_DIVIDER           3.0f
#define BOARD_BAT_CALIBRATION       0.992857f

/* --------------------------------------------------------------- microSD --- */

/* SDMMC slot 1 in 1-bit mode. CLK and CMD are shared with the panel's init
 * SPI, which is only used before the RGB interface takes over, so the two do
 * not collide as long as the display is brought up first. */
#define BOARD_SD_CLK_GPIO           2
#define BOARD_SD_CMD_GPIO           1
#define BOARD_SD_D0_GPIO            42

/* ------------------------------------------------------- IMU orientation --- */

/* Board level axis mapping and polarity for the QMI8658.
 *
 * Index i of *_MAP selects which raw sensor axis feeds logical axis i, and
 * *_SIGN flips it. The demos expect the RP2350 convention: lying flat with the
 * screen up reads +1 g on Z, tilting right is +X, tilting forward is +Y.
 *
 * Measured on this board, flat and screen up: (+0.05, +0.06, -0.98) g. The
 * sensor's +Z therefore points into the back of the board, so the frame needs
 * turning over. That is a 180 degree rotation, which flips two axes, not one:
 * negating Z alone would leave a mirrored left-handed frame and the fusion
 * filters would wind yaw the wrong way. The rotation below is 180 degrees about
 * X, so Y and Z both invert.
 *
 * The remaining ambiguity is a 180 degree spin about Z, which decides whether
 * tilting the board right leans the cube right or left. If it comes out
 * mirrored, use { -1.0f, 1.0f, -1.0f } for both SIGN lines instead.
 *
 * The gyroscope has to get the same transformation as the accelerometer or the
 * filters see an inconsistent frame. */
#define BOARD_IMU_ACCEL_MAP         { 0, 1, 2 }
#define BOARD_IMU_ACCEL_SIGN        { 1.0f, -1.0f, -1.0f }
#define BOARD_IMU_GYRO_MAP          { 0, 1, 2 }
#define BOARD_IMU_GYRO_SIGN         { 1.0f, -1.0f, -1.0f }

#endif /* BOARD_CONFIG_H */
