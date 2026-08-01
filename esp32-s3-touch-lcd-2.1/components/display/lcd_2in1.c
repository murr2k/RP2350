#include "lcd_2in1.h"

#include <stdbool.h>
#include <string.h>

#include "board_config.h"
#include "dev_config.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_log.h"
#include "tca9554.h"

static const char *TAG = "lcd";

LCD_2IN1_ATTRIBUTES LCD_2IN1;

static esp_lcd_panel_handle_t s_panel;
static uint16_t *s_fb[2];
static int s_back;              /* index of the buffer applications draw into */

/* --- ST7701S register sequence -------------------------------------------- */

/* Sent over a 9-bit SPI frame: one leading bit selects command (0) or data (1),
 * then eight bits of payload. The ESP32 SPI master gives us that for free with
 * command_bits = 1 and address_bits = 8, no bit banging required.
 *
 * Values are Waveshare's power-on sequence for this panel. */

typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t len;
    uint16_t delay_ms;
} st7701_init_cmd_t;

static const st7701_init_cmd_t st7701_init_sequence[] = {
    /* Command2 BK0: panel geometry, porches and gamma. */
    {0xFF, {0x77, 0x01, 0x00, 0x00, 0x10}, 5, 0},
    {0xC0, {0x3B, 0x00}, 2, 0},                     /* scan line count */
    {0xC1, {0x0B, 0x02}, 2, 0},                     /* vertical back porch */
    {0xC2, {0x07, 0x02}, 2, 0},
    {0xCC, {0x10}, 1, 0},
    {0xCD, {0x08}, 1, 0},                           /* RGB format */
    {0xB0, {0x00, 0x11, 0x16, 0x0E, 0x11, 0x06, 0x05, 0x09,
            0x08, 0x21, 0x06, 0x13, 0x10, 0x29, 0x31, 0x18}, 16, 0},
    {0xB1, {0x00, 0x11, 0x16, 0x0E, 0x11, 0x07, 0x05, 0x09,
            0x09, 0x21, 0x05, 0x13, 0x11, 0x2A, 0x31, 0x18}, 16, 0},

    /* Command2 BK1: supply rails and timing. */
    {0xFF, {0x77, 0x01, 0x00, 0x00, 0x11}, 5, 0},
    {0xB0, {0x6D}, 1, 0},                           /* VOP */
    {0xB1, {0x37}, 1, 0},                           /* VCOM */
    {0xB2, {0x81}, 1, 0},                           /* VGH 12 V */
    {0xB3, {0x80}, 1, 0},
    {0xB5, {0x43}, 1, 0},                           /* VGL -8.3 V */
    {0xB7, {0x85}, 1, 0},
    {0xB8, {0x20}, 1, 0},
    {0xC1, {0x78}, 1, 0},
    {0xC2, {0x78}, 1, 0},
    {0xD0, {0x88}, 1, 0},
    {0xE0, {0x00, 0x00, 0x02}, 3, 0},
    {0xE1, {0x03, 0xA0, 0x00, 0x00, 0x04, 0xA0, 0x00, 0x00,
            0x00, 0x20, 0x20}, 11, 0},
    {0xE2, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00}, 13, 0},
    {0xE3, {0x00, 0x00, 0x11, 0x00}, 4, 0},
    {0xE4, {0x22, 0x00}, 2, 0},
    {0xE5, {0x05, 0xEC, 0xA0, 0xA0, 0x07, 0xEE, 0xA0, 0xA0,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 16, 0},
    {0xE6, {0x00, 0x00, 0x11, 0x00}, 4, 0},
    {0xE7, {0x22, 0x00}, 2, 0},
    {0xE8, {0x06, 0xED, 0xA0, 0xA0, 0x08, 0xEF, 0xA0, 0xA0,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 16, 0},
    {0xEB, {0x00, 0x00, 0x40, 0x40, 0x00, 0x00, 0x00}, 7, 0},
    {0xED, {0xFF, 0xFF, 0xFF, 0xBA, 0x0A, 0xBF, 0x45, 0xFF,
            0xFF, 0x54, 0xFB, 0xA0, 0xAB, 0xFF, 0xFF, 0xFF}, 16, 0},
    {0xEF, {0x10, 0x0D, 0x04, 0x08, 0x3F, 0x1F}, 6, 0},

    /* Command2 BK3. */
    {0xFF, {0x77, 0x01, 0x00, 0x00, 0x13}, 5, 0},
    {0xEF, {0x08}, 1, 0},

    /* Back to the user command set. */
    {0xFF, {0x77, 0x01, 0x00, 0x00, 0x00}, 5, 0},
    {0x36, {0x00}, 1, 0},                           /* memory access control */
    {0x3A, {0x66}, 1, 0},                           /* pixel format */
    {0x11, {0}, 0, 480},                            /* sleep out */
    {0x20, {0}, 0, 120},                            /* inversion off */
    {0x29, {0}, 0, 0},                              /* display on */
};

static spi_device_handle_t s_spi;

static void st7701_write_command(uint8_t cmd)
{
    spi_transaction_t t = {
        .cmd = 0,
        .addr = cmd,
        .length = 0,
    };
    ESP_ERROR_CHECK(spi_device_polling_transmit(s_spi, &t));
}

static void st7701_write_data(uint8_t data)
{
    spi_transaction_t t = {
        .cmd = 1,
        .addr = data,
        .length = 0,
    };
    ESP_ERROR_CHECK(spi_device_polling_transmit(s_spi, &t));
}

static void st7701_reset(void)
{
    TCA9554_Set(BOARD_EXIO_LCD_RST, 0);
    DEV_Delay_ms(20);
    TCA9554_Set(BOARD_EXIO_LCD_RST, 1);
    DEV_Delay_ms(120);
}

static esp_err_t st7701_send_init_sequence(void)
{
    const spi_bus_config_t bus = {
        .mosi_io_num = BOARD_LCD_SPI_MOSI_GPIO,
        .miso_io_num = -1,
        .sclk_io_num = BOARD_LCD_SPI_CLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64,
    };
    esp_err_t err = spi_bus_initialize(BOARD_LCD_SPI_HOST, &bus, SPI_DMA_DISABLED);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(err));
        return err;
    }

    const spi_device_interface_config_t dev = {
        .command_bits = 1,      /* the data/command select bit */
        .address_bits = 8,      /* the payload byte */
        .mode = 0,
        .clock_speed_hz = BOARD_LCD_SPI_HZ,
        .spics_io_num = -1,     /* chip select lives on the IO expander */
        .queue_size = 1,
    };
    err = spi_bus_add_device(BOARD_LCD_SPI_HOST, &dev, &s_spi);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI device add failed: %s", esp_err_to_name(err));
        spi_bus_free(BOARD_LCD_SPI_HOST);
        return err;
    }

    TCA9554_Set(BOARD_EXIO_LCD_CS, 0);
    DEV_Delay_ms(10);

    for (size_t i = 0; i < sizeof(st7701_init_sequence) / sizeof(st7701_init_sequence[0]); i++) {
        const st7701_init_cmd_t *entry = &st7701_init_sequence[i];
        st7701_write_command(entry->cmd);
        for (uint8_t b = 0; b < entry->len; b++) {
            st7701_write_data(entry->data[b]);
        }
        if (entry->delay_ms) {
            DEV_Delay_ms(entry->delay_ms);
        }
    }

    TCA9554_Set(BOARD_EXIO_LCD_CS, 1);
    DEV_Delay_ms(10);

    /* Nothing else is sent over this bus, and GPIO1/GPIO2 double as the microSD
     * CMD/CLK lines, so hand them back. */
    spi_bus_remove_device(s_spi);
    s_spi = NULL;
    spi_bus_free(BOARD_LCD_SPI_HOST);
    return ESP_OK;
}

/* --- RGB panel ------------------------------------------------------------ */

static esp_err_t rgb_panel_start(void)
{
    const int data_gpios[] = BOARD_LCD_RGB_DATA_GPIOS;

    esp_lcd_rgb_panel_config_t cfg = {
        .clk_src = LCD_CLK_SRC_DEFAULT,
        .timings = {
            .pclk_hz = BOARD_LCD_PCLK_HZ,
            .h_res = BOARD_LCD_WIDTH,
            .v_res = BOARD_LCD_HEIGHT,
            .hsync_pulse_width = BOARD_LCD_HSYNC_PULSE_WIDTH,
            .hsync_back_porch = BOARD_LCD_HSYNC_BACK_PORCH,
            .hsync_front_porch = BOARD_LCD_HSYNC_FRONT_PORCH,
            .vsync_pulse_width = BOARD_LCD_VSYNC_PULSE_WIDTH,
            .vsync_back_porch = BOARD_LCD_VSYNC_BACK_PORCH,
            .vsync_front_porch = BOARD_LCD_VSYNC_FRONT_PORCH,
            .flags = {
                .pclk_active_neg = false,
            },
        },
        .data_width = 16,
        .bits_per_pixel = 16,
        .num_fbs = 2,               /* double buffered, both in PSRAM */
        .bounce_buffer_size_px = 0, /* see the note below */
        .hsync_gpio_num = BOARD_LCD_RGB_HSYNC_GPIO,
        .vsync_gpio_num = BOARD_LCD_RGB_VSYNC_GPIO,
        .de_gpio_num = BOARD_LCD_RGB_DE_GPIO,
        .pclk_gpio_num = BOARD_LCD_RGB_PCLK_GPIO,
        .disp_gpio_num = BOARD_LCD_RGB_DISP_GPIO,
        .flags = {
            .fb_in_psram = true,
        },
    };
    for (int i = 0; i < 16; i++) {
        cfg.data_gpio_nums[i] = data_gpios[i];
    }

    /* No bounce buffer: sdkconfig.defaults keeps code and constants out of
     * flash (SPIRAM_FETCH_INSTRUCTIONS / SPIRAM_RODATA), which is the other
     * documented cure for the panel drifting when the cache misses. If you turn
     * those off, set bounce_buffer_size_px to LCD_2IN1_WIDTH * 10 instead. */

    esp_err_t err = esp_lcd_new_rgb_panel(&cfg, &s_panel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "RGB panel alloc failed: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_lcd_panel_reset(s_panel);
    if (err == ESP_OK) {
        err = esp_lcd_panel_init(s_panel);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "RGB panel init failed: %s", esp_err_to_name(err));
        return err;
    }

    void *fb0 = NULL;
    void *fb1 = NULL;
    err = esp_lcd_rgb_panel_get_frame_buffer(s_panel, 2, &fb0, &fb1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "no frame buffers: %s", esp_err_to_name(err));
        return err;
    }
    s_fb[0] = (uint16_t *)fb0;
    s_fb[1] = (uint16_t *)fb1;
    s_back = 1;
    return ESP_OK;
}

/* --- public API ----------------------------------------------------------- */

esp_err_t LCD_2IN1_Init(uint8_t Scan_dir)
{
    LCD_2IN1.WIDTH = LCD_2IN1_WIDTH;
    LCD_2IN1.HEIGHT = LCD_2IN1_HEIGHT;
    LCD_2IN1.SCAN_DIR = Scan_dir;

    const uint8_t backlight = DEV_GET_PWM();
    DEV_SET_PWM(0);

    st7701_reset();

    esp_err_t err = st7701_send_init_sequence();
    if (err != ESP_OK) {
        return err;
    }

    err = rgb_panel_start();
    if (err != ESP_OK) {
        return err;
    }

    LCD_2IN1_Clear(0x0000);
    DEV_SET_PWM(backlight);

    ESP_LOGI(TAG, "ST7701S up: %dx%d, %d MHz pixel clock, 2 frame buffers in PSRAM",
             LCD_2IN1_WIDTH, LCD_2IN1_HEIGHT, BOARD_LCD_PCLK_HZ / 1000000);
    return ESP_OK;
}

uint16_t *LCD_2IN1_GetBuffer(void)
{
    return s_fb[s_back];
}

void LCD_2IN1_Display(uint16_t *Image)
{
    if (s_panel == NULL || Image == NULL) {
        return;
    }

    esp_lcd_panel_draw_bitmap(s_panel, 0, 0, LCD_2IN1_WIDTH, LCD_2IN1_HEIGHT, Image);

    /* Handing back one of the panel's own buffers flips which one is scanned
     * out, so the other one becomes the drawing target. Any other pointer was
     * copied into the live buffer and nothing changes hands. */
    if (Image == s_fb[0]) {
        s_back = 1;
    } else if (Image == s_fb[1]) {
        s_back = 0;
    }
}

void LCD_2IN1_Clear(uint16_t Color)
{
    if (s_panel == NULL) {
        return;
    }
    for (int i = 0; i < 2; i++) {
        uint16_t *fb = s_fb[i];
        if (fb == NULL) {
            continue;
        }
        for (int p = 0; p < LCD_2IN1_WIDTH * LCD_2IN1_HEIGHT; p++) {
            fb[p] = Color;
        }
        LCD_2IN1_Display(fb);
    }
}

void LCD_2IN1_DisplayWindows(uint16_t Xstart, uint16_t Ystart, uint16_t Xend, uint16_t Yend,
                             uint16_t *Image)
{
    if (s_panel == NULL || Image == NULL) {
        return;
    }
    if (Xend >= LCD_2IN1_WIDTH) {
        Xend = LCD_2IN1_WIDTH - 1;
    }
    if (Yend >= LCD_2IN1_HEIGHT) {
        Yend = LCD_2IN1_HEIGHT - 1;
    }
    if (Xstart > Xend || Ystart > Yend) {
        return;
    }

    /* esp_lcd wants the source rows packed for the rectangle being drawn, while
     * the caller hands us a full-size image, so feed it a row at a time. */
    const int width = Xend - Xstart + 1;
    uint16_t row[LCD_2IN1_WIDTH];

    for (int y = Ystart; y <= Yend; y++) {
        memcpy(row, &Image[y * LCD_2IN1_WIDTH + Xstart], (size_t)width * sizeof(uint16_t));
        esp_lcd_panel_draw_bitmap(s_panel, Xstart, y, Xstart + width, y + 1, row);
    }
}

void LCD_2IN1_DisplayPoint(uint16_t X, uint16_t Y, uint16_t Color)
{
    if (s_panel == NULL || X >= LCD_2IN1_WIDTH || Y >= LCD_2IN1_HEIGHT) {
        return;
    }
    esp_lcd_panel_draw_bitmap(s_panel, X, Y, X + 1, Y + 1, &Color);
}

void LCD_2IN1_SetBacklight(uint8_t percent)
{
    DEV_SET_PWM(percent);
}
