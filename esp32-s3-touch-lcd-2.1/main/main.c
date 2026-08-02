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
#include "net_time.h"
#include "rtc_pcf85063.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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
    &demo_finger_cube,
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
    &demo_rain,
};

/* The picker hands over to the screensaver after this long with nothing
 * touched or typed. Any contact brings it back. */
#define SCREENSAVER_IDLE_US 30000000ULL

#define DEMO_COUNT ((int)(sizeof(s_demos) / sizeof(s_demos[0])))

/* The picker is a drum: one continuous scroll position, snapped to a detent on
 * release, with whatever sits in the middle being the selection. That is the
 * round screen convention, from Wear OS's ScalingLazyColumn and LVGL's roller:
 * the centre is the only place with full width, so the centre is the cursor.
 *
 * Items shrink and dim with distance from the middle. Our font only comes in
 * whole scales, so "shrink" is three tiers rather than a smooth curve, and they
 * are also pushed outward along the bezel arc, which is what Wear OS's curving
 * layout does. */
#define CAROUSEL_PITCH      84      /* pixels between detents */
#define CAROUSEL_FRICTION   7.0f    /* per second, momentum decay */
#define CAROUSEL_SNAP       14.0f   /* per second, pull toward the nearest item */
#define CAROUSEL_TAP_SLOP   12      /* pixels of movement still counted as a tap */
#define CAROUSEL_MAX_FLING  7.0f    /* items per second */

/* How far a fling carries past the finger, in items, is MAX_FLING / FRICTION,
 * because the speed decays by a constant fraction per second and the distance
 * is its integral. One item, here: enough to feel like it was thrown, not
 * enough to lose your place.
 *
 * Being sensitive to a flick is a separate matter from carrying far, and was
 * the bigger half of the problem. Speed used to come from a single pair of
 * touch samples, so one jittery reading a few pixels wide became a fling of
 * several items. It is smoothed now, and a finger that comes to rest before
 * lifting parks the list instead of throwing it. */
#define CAROUSEL_VEL_TAU    0.05f   /* seconds, smoothing on the drag speed */
#define CAROUSEL_STILL_US   120000  /* still for this long: a park, not a flick */

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

/* The RGB panel's interrupt is allocated on whichever core creates the panel,
 * and in bounce mode that ISR copies the whole 450 KB frame out of PSRAM every
 * refresh. Creating the panel from core 1 keeps that work off the core doing
 * the drawing. */

static esp_err_t s_lcd_init_result = ESP_FAIL;
static volatile bool s_lcd_init_done;

static void lcd_init_task(void *arg)
{
    (void)arg;
    s_lcd_init_result = LCD_2IN1_Init(HORIZONTAL);
    s_lcd_init_done = true;
    vTaskDelete(NULL);
}

static esp_err_t lcd_init_on_sensor_core(void)
{
    if (xTaskCreatePinnedToCore(lcd_init_task, "lcd_init", 4096, NULL, 5, NULL, 1) != pdPASS) {
        return LCD_2IN1_Init(HORIZONTAL);    /* fall back to this core */
    }
    while (!s_lcd_init_done) {
        demo_delay_ms(10);
    }
    return s_lcd_init_result;
}

static void print_menu(void)
{
    printf("\n=== ESP32-S3-Touch-LCD-2.1 demo collection ===\n");
    printf("Work-alike of the RP2350-LCD-1.28 demos, one key per demo:\n\n");
    for (int i = 0; i < DEMO_COUNT; i++) {
        printf("  %c  %-22s %s\n", menu_key(i), s_demos[i]->name, s_demos[i]->summary);
    }
    printf("\n  drag       scroll the picker, tap the middle entry to start\n");
    printf("  j / k      move the picker down / up\n");
    printf("  Enter      start whatever is centred\n");
    printf("  ESC or ~   leave a running demo\n");
    printf("  ?          reprint this menu\n\n");
}

static float clampf(float v, float lo, float hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

/* The picker reads as a list of things rather than a list of symbols, so the
 * underscores come out and the letters go up. Only for display: the name itself
 * stays as it is, because it is the RP2350 target name and the console menu,
 * the key lookup and the log lines all still use it. */
static const char *menu_label(int index, char *buf, size_t len)
{
    const char *name = s_demos[index]->name;
    size_t i = 0;

    for (; name[i] != '\0' && i + 1 < len; i++) {
        const char c = name[i];
        buf[i] = (c == '_')                 ? ' '
               : (c >= 'a' && c <= 'z')     ? (char)(c - 'a' + 'A')
                                            : c;
    }
    buf[i] = '\0';
    return buf;
}

/* The clock and the date, across the top above the list.
 *
 * Read from the part rather than from the system clock, so what is on screen is
 * what the RTC holds. That is the point of setting it, and once a backup cell is
 * fitted it will be the only one of the two that survives a power cycle.
 *
 * The part is only read when the second it is showing has run out, not once a
 * frame: at 40 fps that would be 40 transfers a second onto a bus the sensor
 * task already has at 250 Hz, to show a number that changes once. */
#define CLOCK_Y     26          /* top of the time, scale 3, 21 px tall */
#define DATE_Y      54          /* top of the line under it, scale 2, 14 px */

static void draw_clock(void)
{
    static const char *const days[7] = {"SUN", "MON", "TUE", "WED",
                                        "THU", "FRI", "SAT"};
    static const char *const months[12] = {"JAN", "FEB", "MAR", "APR", "MAY", "JUN",
                                           "JUL", "AUG", "SEP", "OCT", "NOV", "DEC"};
    static pcf85063_time_t shown;
    static uint64_t read_us;
    static bool have;

    const uint64_t now = demo_micros();
    if (!have || now - read_us >= 1000000ULL) {
        have = pcf85063_get(&shown) && pcf85063_running();
        read_us = now;
    }

    char text[32];

    if (!have) {
        /* Nothing worth showing yet, so the space below says what it is waiting
         * on rather than sitting blank or showing a date of nowhere. */
        const bool failed = net_time_state() == NET_TIME_FAILED;
        gfx_text_centered(DISP_CX, CLOCK_Y, "--:--:--",
                          failed ? GFX_DGREY : gfx_dim(GFX_WHITE, 6, 16), 3);
        snprintf(text, sizeof(text), "TIME %s", net_time_status());
        gfx_text_centered(DISP_CX, DATE_Y, text, failed ? GFX_DGREY : GFX_GREY, 2);
        return;
    }

    snprintf(text, sizeof(text), "%02u:%02u:%02u",
             shown.hour, shown.minute, shown.second);
    gfx_text_centered(DISP_CX, CLOCK_Y, text, GFX_WHITE, 3);

    /* Both indices come off the part, so neither is trusted to be in range: a
     * clock that has lost power reads back whatever it likes. */
    const char *day = (shown.weekday < 7) ? days[shown.weekday] : "---";
    const char *month = (shown.month >= 1 && shown.month <= 12)
                            ? months[shown.month - 1] : "---";
    snprintf(text, sizeof(text), "%s %02u %s %04u",
             day, shown.day, month, shown.year);
    gfx_text_centered(DISP_CX, DATE_Y, text, GFX_GREY, 2);
}

static void draw_carousel(float scroll)
{
    demo_frame_begin(GFX_BLACK);
    draw_clock();

    /* The detent, drawn faintly so the centre reads as the selection without
     * needing a highlight bar. */
    const uint16_t guide = GFX_RGB(0, 60, 80);
    gfx_hline(DISP_CX - 140, DISP_CY - CAROUSEL_PITCH / 2, 280, guide);
    gfx_hline(DISP_CX - 140, DISP_CY + CAROUSEL_PITCH / 2, 280, guide);

    for (int i = 0; i < DEMO_COUNT; i++) {
        const int dy = (int)(((float)i - scroll) * CAROUSEL_PITCH);
        const int distance = abs(dy);

        /* Downward the list runs out at the bezel. Upward it has to be gone
         * before the date, so it fades over a shorter run rather than being cut
         * off at a line: an item that climbed to the top of the screen would
         * otherwise arrive over the clock, faint but there, at whichever scroll
         * positions happened to put it there. */
        const int reach = (dy < 0) ? (DISP_CY - DATE_Y - 22) : 230;
        if (distance > reach) {
            continue;
        }

        /* Three tiers, because the font only comes in whole scales. */
        int scale;
        int level;
        if (distance < CAROUSEL_PITCH / 2) {
            scale = 3;
            level = 16;
        } else if (distance < CAROUSEL_PITCH * 3 / 2) {
            scale = 2;
            level = 10;
        } else {
            scale = 1;
            level = 6;
        }

        /* Fade into the bezel, so the ends of the list dissolve rather than
         * being clipped by the glass. */
        const int fade = 16 - (distance * 16) / reach;
        if (fade < level) {
            level = fade;
        }
        if (level <= 0) {
            continue;
        }

        const uint16_t color = gfx_dim((distance < CAROUSEL_PITCH / 2) ? GFX_CYAN : GFX_WHITE,
                                       level, 16);
        char label[40];
        gfx_text_centered(DISP_CX, DISP_CY + dy - gfx_text_height(scale) / 2,
                          menu_label(i, label, sizeof(label)), color, scale);
    }

    /* Just below the lower detent line, which keeps it clear of the next item
     * down however far apart the rows are set. */
    const int centred = (int)clampf(scroll + 0.5f, 0.0f, (float)(DEMO_COUNT - 1));
    gfx_text_centered(DISP_CX, DISP_CY + CAROUSEL_PITCH / 2 + 12,
                      s_demos[centred]->summary, GFX_DGREY, 1);

    char status[80];
    snprintf(status, sizeof(status), "IMU %s   TOUCH %s   TIME %s   BAT %.2fV",
             qmi8658_present() ? "ok" : "--",
             cst820_present() ? "ok" : "--",
             net_time_status(),
             DEV_Battery_Volts());
    gfx_text_centered(DISP_CX, DISP_H - 62, status, GFX_GREY, 1);
    gfx_text_centered(DISP_CX, DISP_H - 48, "drag to scroll, tap the middle to start",
                      gfx_dim(GFX_GREY, 8, 16), 1);

    demo_frame_end();
}

/** Block until the user picks a demo, and return its index. */
static int menu_select(void)
{
    static float s_scroll;          /* survives between demos, so the picker
                                     * comes back where it was left */
    float velocity = 0.0f;
    float seek_target = -1.0f;

    bool dragging = false;
    int drag_start_y = 0;
    float drag_start_scroll = 0.0f;
    int drag_travel = 0;
    int last_y = 0;
    uint64_t last_move_us = 0;
    int press_y = DISP_CY;      /* last position while actually touched: the
                                 * release event carries no coordinates */

    /* A demo is usually left by holding a finger on the glass, so on the way
     * back in the finger is still down. Wait for it to lift before listening,
     * or that same contact is read as a tap and relaunches what was just
     * exited. */
    bool armed = !cst820_present();
    uint64_t last_sample_us = 0;
    uint64_t last_frame_us = demo_micros();
    uint64_t last_input_us = demo_micros();
    int announced = -1;

    print_menu();

    for (;;) {
        int c;
        while ((c = demo_read_char()) >= 0) {
            last_input_us = demo_micros();
            if (c == '?') {
                print_menu();
                continue;
            }
            if (c == '\r' || c == '\n') {
                return (int)clampf(s_scroll + 0.5f, 0.0f, (float)(DEMO_COUNT - 1));
            }
            if (c == 'j' || c == 'k') {
                /* Keyboard drives the same animation the finger does. */
                const float from = (seek_target >= 0.0f) ? seek_target : s_scroll;
                seek_target = clampf((float)((int)(from + 0.5f) + ((c == 'j') ? 1 : -1)),
                                     0.0f, (float)(DEMO_COUNT - 1));
                velocity = 0.0f;
                continue;
            }
            const int index = menu_index_for_key((char)c);
            if (index >= 0) {
                s_scroll = (float)index;
                return index;
            }
        }

        const uint64_t now = demo_micros();
        float dt = (float)(now - last_frame_us) / 1000000.0f;
        last_frame_us = now;
        if (dt > 0.1f) {
            dt = 0.1f;
        }

        touch_state_t touch;
        bool pressed = demo_touch(&touch);
        if (pressed) {
            last_input_us = now;
            press_y = touch.y;
        }

        if (!armed) {
            /* Still carrying the contact that left the last demo. */
            if (!pressed) {
                armed = true;
            }
            pressed = false;
        }

        if (now - last_input_us > SCREENSAVER_IDLE_US) {
            demo_clear_exit();
            demo_rain.run();
            demo_clear_exit();
            last_input_us = demo_micros();
            last_frame_us = last_input_us;
            announced = -1;
            dragging = false;
            velocity = 0.0f;
            armed = !cst820_present();  /* the dismissing touch is still down */
            continue;
        }

        if (pressed && !dragging) {
            dragging = true;
            drag_start_y = touch.y;
            drag_start_scroll = s_scroll;
            drag_travel = 0;
            last_y = touch.y;
            last_sample_us = now;
            last_move_us = now;
            velocity = 0.0f;
            seek_target = -1.0f;
        } else if (pressed) {
            /* The list follows the finger: dragging down brings earlier items
             * into view, so scroll runs the other way. */
            s_scroll = clampf(drag_start_scroll -
                                  (float)(touch.y - drag_start_y) / CAROUSEL_PITCH,
                              0.0f, (float)(DEMO_COUNT - 1));

            const int moved = touch.y - last_y;
            if (abs(touch.y - drag_start_y) > drag_travel) {
                drag_travel = abs(touch.y - drag_start_y);
            }
            const float sample_dt = (float)(now - last_sample_us) / 1000000.0f;
            if (moved != 0 && sample_dt > 0.0005f) {
                /* Eased towards, not taken outright, so a single stray sample
                 * cannot become a fling on its own. */
                const float instant = -(float)moved / CAROUSEL_PITCH / sample_dt;
                velocity += (instant - velocity) *
                            (sample_dt / (CAROUSEL_VEL_TAU + sample_dt));
                last_y = touch.y;
                last_sample_us = now;
                last_move_us = now;
            }
        } else if (dragging) {
            dragging = false;

            if (drag_travel <= CAROUSEL_TAP_SLOP) {
                /* A tap. On the centred item it starts; anywhere else it brings
                 * that item to the middle, which is forgiving of near misses.
                 *
                 * The position comes from the last frame the finger was still
                 * down: the release itself reports no coordinates, and reading
                 * them anyway put every tap three items above where it was. */
                velocity = 0.0f;
                const int centred = (int)clampf(s_scroll + 0.5f, 0.0f,
                                               (float)(DEMO_COUNT - 1));
                const int tapped = (int)clampf(
                    s_scroll + (float)(press_y - DISP_CY) / CAROUSEL_PITCH + 0.5f,
                    0.0f, (float)(DEMO_COUNT - 1));
                if (tapped == centred) {
                    s_scroll = (float)centred;
                    return centred;
                }
                seek_target = (float)tapped;
            } else if (now - last_move_us > CAROUSEL_STILL_US) {
                /* Held still and then lifted. That is putting the list down,
                 * not throwing it, whatever it was doing on the way here. */
                velocity = 0.0f;
            } else {
                velocity = clampf(velocity, -CAROUSEL_MAX_FLING, CAROUSEL_MAX_FLING);
            }
        }

        if (!dragging) {
            if (velocity > 0.05f || velocity < -0.05f) {
                s_scroll += velocity * dt;
                velocity -= velocity * CAROUSEL_FRICTION * dt;
                if (s_scroll < 0.0f || s_scroll > (float)(DEMO_COUNT - 1)) {
                    s_scroll = clampf(s_scroll, 0.0f, (float)(DEMO_COUNT - 1));
                    velocity = 0.0f;
                }
            } else {
                velocity = 0.0f;
                const float target = (seek_target >= 0.0f) ? seek_target
                                                           : (float)(int)(s_scroll + 0.5f);
                const float delta = target - s_scroll;
                s_scroll += delta * clampf(CAROUSEL_SNAP * dt, 0.0f, 1.0f);
                if (delta < 0.01f && delta > -0.01f) {
                    s_scroll = target;
                    seek_target = -1.0f;
                }
            }
        }

        const int centred = (int)clampf(s_scroll + 0.5f, 0.0f, (float)(DEMO_COUNT - 1));
        if (centred != announced) {
            announced = centred;
            printf("> %c  %s\n", menu_key(centred), s_demos[centred]->name);
        }

        draw_carousel(s_scroll);

        /* The exit gesture has no meaning here, and the flag is cleared before
         * the next demo starts, so the picker ignores it. Calling
         * demo_clear_exit() in this loop would also flush the key just typed. */
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

    if (lcd_init_on_sensor_core() != ESP_OK) {
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
    if (!pcf85063_init()) {
        ESP_LOGW(TAG, "clock missing, the picker will not show a time");
    }

    /* Sensor acquisition and the demos' filters live on the other core, so the
     * frame budget carries only drawing. */
    demo_sensor_start();

    /* Goes off and fetches the time in the background, then puts the radio back
     * down. The picker is up and usable throughout. */
    net_time_start();

    for (;;) {
        const int index = menu_select();
        run_demo(s_demos[index]);
    }
}
