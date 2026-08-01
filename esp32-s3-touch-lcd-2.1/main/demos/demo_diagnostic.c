/**
 * Board diagnostic.
 *
 * Work-alike of src/rp2350_diagnostic.c: walk the hardware step by step and
 * report what answered, both on the serial console and on the panel. Where the
 * RP2350 version poked the LCD registers by hand, this one reports the buses
 * that the 2.1" board actually depends on.
 */

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "board_config.h"
#include "demo_common.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_heap_caps.h"
#include "esp_psram.h"
#include "esp_system.h"
#include "tca9554.h"

#define MAX_LINES 18

static char s_lines[MAX_LINES][48];
static int s_line_count;

static void report(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

static void report(const char *fmt, ...)
{
    char buf[48];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    printf("%s\n", buf);
    if (s_line_count < MAX_LINES) {
        strncpy(s_lines[s_line_count], buf, sizeof(s_lines[0]) - 1);
        s_lines[s_line_count][sizeof(s_lines[0]) - 1] = '\0';
        s_line_count++;
    }
}

static void draw_report(void)
{
    demo_frame_begin(GFX_BLACK);
    gfx_text_centered(DISP_CX, 40, "DIAGNOSTIC", GFX_CYAN, 2);

    for (int i = 0; i < s_line_count; i++) {
        const uint16_t color = (strstr(s_lines[i], "MISSING") != NULL) ? GFX_RED : GFX_WHITE;
        gfx_text(70, 74 + i * 20, s_lines[i], color, 2);
    }
    demo_draw_exit_hint();
    demo_frame_end();
}

static void scan_i2c(void)
{
    char found[48] = {0};
    int count = 0;

    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        if (!DEV_I2C_Probe(addr)) {
            continue;
        }
        count++;
        char entry[8];
        snprintf(entry, sizeof(entry), " %02x", addr);
        if (strlen(found) + strlen(entry) < sizeof(found) - 1) {
            strcat(found, entry);
        }
    }
    report("i2c devices %d:%s", count, found);
}

static void run(void)
{
    s_line_count = 0;

    printf("\n=== ESP32-S3-Touch-LCD-2.1 diagnostic ===\n");

    esp_chip_info_t chip;
    esp_chip_info(&chip);
    report("chip esp32-s3 rev %d, %d core", chip.revision / 100, chip.cores);

    uint32_t flash_size = 0;
    if (esp_flash_get_size(NULL, &flash_size) == ESP_OK) {
        report("flash %lu MB", (unsigned long)(flash_size / (1024 * 1024)));
    } else {
        report("flash size unknown");
    }

    const size_t psram = esp_psram_get_size();
    if (psram == 0) {
        report("psram MISSING");
    } else {
        report("psram %u MB", (unsigned)(psram / (1024 * 1024)));
    }

    report("heap int %u KB, psram %u KB",
           (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024),
           (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024));

    scan_i2c();

    report("expander 0x%02x %s", BOARD_TCA9554_ADDR,
           DEV_I2C_Probe(BOARD_TCA9554_ADDR) ? "ok" : "MISSING");
    report("outputs 0x%02x", TCA9554_ReadReg(TCA9554_REG_OUTPUT));
    report("imu 0x%02x %s", qmi8658_address(),
           qmi8658_present() ? "ok" : "MISSING");
    report("touch 0x%02x %s", BOARD_TOUCH_ADDR,
           cst820_present() ? "ok" : "MISSING");
    report("rtc 0x%02x %s", BOARD_RTC_ADDR,
           DEV_I2C_Probe(BOARD_RTC_ADDR) ? "ok" : "MISSING");
    report("lcd %dx%d, %d MHz pclk", DISP_W, DISP_H, BOARD_LCD_PCLK_HZ / 1000000);
    report("battery %.2f V", (double)DEV_Battery_Volts());
    if (qmi8658_present()) {
        report("imu temp %.1f C", (double)qmi8658_read_temperature());
    }

    draw_report();

    /* Then keep a live readout going until the operator leaves. */
    while (!demo_exit_requested()) {
        vector3f_t acc;
        vector3f_t gyro;
        const bool ok = demo_read_imu(&acc, &gyro);

        demo_frame_begin(GFX_BLACK);
        gfx_text_centered(DISP_CX, 40, "DIAGNOSTIC", GFX_CYAN, 2);
        for (int i = 0; i < s_line_count; i++) {
            const uint16_t color =
                (strstr(s_lines[i], "MISSING") != NULL) ? GFX_RED : GFX_WHITE;
            gfx_text(70, 74 + i * 20, s_lines[i], color, 2);
        }

        if (ok) {
            gfx_printf(70, 74 + s_line_count * 20, GFX_GREEN, 2,
                       "acc %+.2f %+.2f %+.2f", (double)acc.x, (double)acc.y, (double)acc.z);
            gfx_printf(70, 74 + (s_line_count + 1) * 20, GFX_GREEN, 2,
                       "gyr %+6.1f %+6.1f %+6.1f", (double)gyro.x, (double)gyro.y,
                       (double)gyro.z);
        }

        touch_state_t touch;
        if (demo_touch(&touch)) {
            gfx_printf(70, 74 + (s_line_count + 2) * 20, GFX_YELLOW, 2,
                       "touch %3u %3u", touch.x, touch.y);
        }

        demo_draw_exit_hint();
        demo_frame_end();
        demo_delay_ms(100);
    }
}

const demo_t demo_diagnostic = {
    .name = "diagnostic",
    .summary = "bus scan and hardware report",
    .run = run,
};
