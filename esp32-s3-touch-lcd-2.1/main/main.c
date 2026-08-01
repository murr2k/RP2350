/**
 * ESP32-S3-Touch-LCD-2.1 demo launcher.
 *
 * The RP2350 project built one .uf2 per demo and you picked one by dragging it
 * onto the bootloader drive. ESP-IDF links a single image, so every demo is in
 * this firmware and you pick one at runtime: tap it on the touch screen, or
 * press its key on the USB serial console.
 *
 * Inside a demo, hold a finger near the top of the screen for about half a
 * second, or press ESC (or '~') on the console, to come back here.
 */

#include <stdio.h>
#include <string.h>

#include "board_config.h"
#include "demo_common.h"
#include "dev_config.h"
#include "esp_log.h"
#include "esp_system.h"
#include "sdkconfig.h"

#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
#include "driver/usb_serial_jtag.h"
#else
#include <fcntl.h>
#include <unistd.h>
#endif

static const char *TAG = "main";

static const demo_t *const s_demos[] = {
    &demo_display_test,
    &demo_buffered_cube,
    &demo_intuitive_cube,
    &demo_gravity_locked_cube,
    &demo_madgwick_cube,
    &demo_configurable_cube,
    &demo_rotation_test,
    &demo_axis_test,
    &demo_kalman_6dof,
    &demo_kalman_6dof_config,
    &demo_touch_test,
    &demo_diagnostic,
};

#define DEMO_COUNT ((int)(sizeof(s_demos) / sizeof(s_demos[0])))

#define MENU_FIRST_ROW_Y 96
#define MENU_ROW_PITCH   26

/* Keys 1..9 then a, b, c for the rest. */
static char menu_key(int index)
{
    return (index < 9) ? (char)('1' + index) : (char)('a' + index - 9);
}

static int menu_index_for_key(char key)
{
    for (int i = 0; i < DEMO_COUNT; i++) {
        if (menu_key(i) == key) {
            return i;
        }
    }
    return -1;
}

static void console_init(void)
{
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    const esp_err_t err = usb_serial_jtag_driver_install(&cfg);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "console input unavailable: %s", esp_err_to_name(err));
    }
#else
    setvbuf(stdin, NULL, _IONBF, 0);
    const int flags = fcntl(STDIN_FILENO, F_GETFL);
    if (flags >= 0) {
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    }
#endif
    setvbuf(stdout, NULL, _IONBF, 0);
}

static void print_menu(void)
{
    printf("\n=== ESP32-S3-Touch-LCD-2.1 demo collection ===\n");
    printf("Work-alike of the RP2350-LCD-1.28 demos, one key per demo:\n\n");
    for (int i = 0; i < DEMO_COUNT; i++) {
        printf("  %c  %-22s %s\n", menu_key(i), s_demos[i]->name, s_demos[i]->summary);
    }
    printf("\n  ESC or ~   leave a running demo\n");
    printf("  ?          reprint this menu\n\n");
}

static void draw_menu(int highlight)
{
    demo_frame_begin(GFX_BLACK);

    gfx_text_centered(DISP_CX, 46, "ESP32-S3 LCD DEMOS", GFX_CYAN, 2);
    gfx_text_centered(DISP_CX, 70, "tap an entry, or press its key", GFX_DGREY, 1);

    for (int i = 0; i < DEMO_COUNT; i++) {
        const int y = MENU_FIRST_ROW_Y + i * MENU_ROW_PITCH;
        const bool on = (i == highlight);
        const uint16_t color = on ? GFX_BLACK : GFX_WHITE;

        if (on) {
            gfx_fill_rect(DISP_CX - 150, y - 5, 300, MENU_ROW_PITCH - 2, GFX_CYAN);
        }
        char line[40];
        snprintf(line, sizeof(line), "%c %s", menu_key(i), s_demos[i]->name);
        gfx_text(DISP_CX - 140, y, line, color, 2);
    }

    char status[64];
    snprintf(status, sizeof(status), "IMU %s   TOUCH %s   BAT %.2fV",
             qmi8658_present() ? "ok" : "--",
             cst820_present() ? "ok" : "--",
             DEV_Battery_Volts());
    gfx_text_centered(DISP_CX, DISP_H - 62, status, GFX_GREY, 1);

    demo_frame_end();
}

/** Block until the user picks a demo, and return its index. */
static int menu_select(void)
{
    int highlight = -1;
    bool was_pressed = false;
    int pressed_row = -1;

    draw_menu(highlight);
    print_menu();

    for (;;) {
        const int c = demo_read_char();
        if (c >= 0) {
            if (c == '?') {
                print_menu();
            } else {
                const int index = menu_index_for_key((char)c);
                if (index >= 0) {
                    return index;
                }
            }
        }

        touch_state_t touch;
        const bool pressed = demo_touch(&touch);

        if (pressed) {
            const int row = (touch.y - MENU_FIRST_ROW_Y + 5) / MENU_ROW_PITCH;
            const int valid = (row >= 0 && row < DEMO_COUNT &&
                               touch.y >= MENU_FIRST_ROW_Y - 5) ? row : -1;
            if (valid != highlight) {
                highlight = valid;
                draw_menu(highlight);
            }
            pressed_row = valid;
            was_pressed = true;
        } else if (was_pressed) {
            /* Launch on release, so a drag can be used to change your mind. */
            was_pressed = false;
            if (pressed_row >= 0) {
                return pressed_row;
            }
            highlight = -1;
            draw_menu(highlight);
        }

        /* The exit gesture has no meaning here, and the flag is cleared before
         * the next demo starts, so the menu simply ignores it. Calling
         * demo_clear_exit() in this loop would also flush the key that was
         * just typed. */

        demo_delay_ms(20);
    }
}

static void run_demo(const demo_t *demo)
{
    printf("\n--- %s: %s ---\n", demo->name, demo->summary);
    ESP_LOGI(TAG, "starting %s", demo->name);

    demo_clear_exit();
    demo->run();
    demo_set_filter(NULL);      /* in case the demo returned early */
    demo_clear_exit();

    printf("\n--- %s finished, back to the menu ---\n", demo->name);
}

void app_main(void)
{
    console_init();

    printf("\n\nWaveshare ESP32-S3-Touch-LCD-2.1\n");
    printf("LCD 480x480 ST7701S, QMI8658 IMU, CST820 touch\n");

    if (DEV_Module_Init() != 0) {
        ESP_LOGE(TAG, "board init failed, nothing else can run");
        for (;;) {
            demo_delay_ms(1000);
        }
    }

    if (LCD_2IN1_Init(HORIZONTAL) != ESP_OK) {
        ESP_LOGE(TAG, "display init failed");
        for (;;) {
            demo_delay_ms(1000);
        }
    }
    DEV_SET_PWM(100);

    if (!cst820_init()) {
        ESP_LOGW(TAG, "touch controller missing, use the serial console instead");
    }
    if (!qmi8658_init()) {
        ESP_LOGW(TAG, "IMU missing, the motion demos will show a warning");
    }

    /* Sensor acquisition and the demos' filters live on the other core, so the
     * frame budget carries only drawing. */
    demo_sensor_start();

    for (;;) {
        const int index = menu_select();
        run_demo(s_demos[index]);
    }
}
