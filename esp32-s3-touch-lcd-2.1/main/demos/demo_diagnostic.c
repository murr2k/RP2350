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
        gfx_text_centered(DISP_CX, 74 + i * 20, s_lines[i], color, 2);
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

/* Temporary: what does a full-frame pass over PSRAM actually cost, and how much
 * of it is the cache fetching lines we are about to overwrite? */
static void bench_psram(void)
{
    volatile uint32_t *p = (volatile uint32_t *)LCD_2IN1_GetBuffer();
    const size_t bytes = (size_t)LCD_2IN1_WIDTH * LCD_2IN1_HEIGHT * sizeof(uint16_t);
    const size_t words = bytes / 4;
    const double mb = (double)bytes / (1024.0 * 1024.0);
    uint64_t t;

    t = demo_micros();
    memset((void *)p, 0, bytes);
    const uint64_t t_set = demo_micros() - t;

    t = demo_micros();
    uint32_t sum = 0;
    for (size_t i = 0; i < words; i++) {
        sum += p[i];
    }
    const uint64_t t_read = demo_micros() - t;

    t = demo_micros();
    for (size_t i = 0; i < words; i++) {
        p[i] = p[i];
    }
    const uint64_t t_rmw = demo_micros() - t;

    /* One store per 64 byte cache line: same number of lines touched as the
     * memset, but a sixteenth of the stores. */
    t = demo_micros();
    for (size_t i = 0; i < words; i += 16) {
        p[i] = 0;
    }
    const uint64_t t_sparse = demo_micros() - t;

    printf("\n[psram %zu KB, checksum %lu]\n", bytes / 1024, (unsigned long)sum);
    printf("  memset      %6llu us  %5.1f MB/s\n", (unsigned long long)t_set,
           mb / (t_set / 1000000.0));
    printf("  read only   %6llu us  %5.1f MB/s\n", (unsigned long long)t_read,
           mb / (t_read / 1000000.0));
    printf("  read+write  %6llu us  %5.1f MB/s\n", (unsigned long long)t_rmw,
           mb / (t_rmw / 1000000.0));
    printf("  1 store/line%6llu us  %5.1f MB/s of lines\n", (unsigned long long)t_sparse,
           mb / (t_sparse / 1000000.0));

    memset((void *)p, 0, bytes);
}

static void run(void)
{
    s_line_count = 0;

    printf("\n=== ESP32-S3-Touch-LCD-2.1 diagnostic ===\n");
    bench_psram();

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

    report("heap %uK int, %uK psram",
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
            gfx_text_centered(DISP_CX, 74 + i * 20, s_lines[i], color, 2);
        }

        char line[48];
        if (ok) {
            snprintf(line, sizeof(line), "acc %+.2f %+.2f %+.2f",
                     (double)acc.x, (double)acc.y, (double)acc.z);
            gfx_text_centered(DISP_CX, 74 + s_line_count * 20, line, GFX_GREEN, 2);
            snprintf(line, sizeof(line), "gyr %+6.1f %+6.1f %+6.1f",
                     (double)gyro.x, (double)gyro.y, (double)gyro.z);
            gfx_text_centered(DISP_CX, 74 + (s_line_count + 1) * 20, line, GFX_GREEN, 2);
        }

        touch_state_t touch;
        if (demo_touch(&touch)) {
            snprintf(line, sizeof(line), "touch %3u %3u",
                     (unsigned)touch.x, (unsigned)touch.y);
            gfx_text_centered(DISP_CX, 74 + (s_line_count + 2) * 20, line, GFX_YELLOW, 2);
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
