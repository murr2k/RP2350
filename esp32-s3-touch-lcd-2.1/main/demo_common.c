#include "demo_common.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "board_config.h"
#include "dev_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
#include "driver/usb_serial_jtag.h"
#else
#include <fcntl.h>
#include <unistd.h>
#endif

/* --- geometry ------------------------------------------------------------- */

const vertex_t demo_cube_vertices[8] = {
    {-50, -50, -50}, {50, -50, -50}, {50, 50, -50}, {-50, 50, -50},
    {-50, -50,  50}, {50, -50,  50}, {50, 50,  50}, {-50, 50,  50},
};

const int demo_cube_edges[12][2] = {
    {0, 1}, {1, 2}, {2, 3}, {3, 0},     /* back face */
    {4, 5}, {5, 6}, {6, 7}, {7, 4},     /* front face */
    {0, 4}, {1, 5}, {2, 6}, {3, 7},     /* connecting edges */
};

/* --- maths ---------------------------------------------------------------- */

float invSqrt(float x)
{
    union {
        float f;
        uint32_t i;
    } conv;

    const float halfx = 0.5f * x;
    conv.f = x;
    conv.i = 0x5f3759df - (conv.i >> 1);
    conv.f = conv.f * (1.5f - (halfx * conv.f * conv.f));
    return conv.f;
}

void quat_identity(quaternion_t *q)
{
    q->w = 1.0f;
    q->x = 0.0f;
    q->y = 0.0f;
    q->z = 0.0f;
}

void quat_normalize(quaternion_t *q)
{
    const float norm = sqrtf(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
    if (norm > 0.0001f) {
        const float inv = 1.0f / norm;
        q->w *= inv;
        q->x *= inv;
        q->y *= inv;
        q->z *= inv;
    } else {
        quat_identity(q);
    }
}

void quat_integrate(quaternion_t *q, float gx, float gy, float gz, float dt)
{
    const float qw = q->w;
    const float qx = q->x;
    const float qy = q->y;
    const float qz = q->z;
    const float half_dt = 0.5f * dt;

    q->w += half_dt * (-qx * gx - qy * gy - qz * gz);
    q->x += half_dt * (qw * gx + qy * gz - qz * gy);
    q->y += half_dt * (qw * gy - qx * gz + qz * gx);
    q->z += half_dt * (qw * gz + qx * gy - qy * gx);

    quat_normalize(q);
}

void quat_rotate_vertex(vertex_t *v, const quaternion_t *q)
{
    const float qw = q->w, qx = q->x, qy = q->y, qz = q->z;
    const float vx = v->x, vy = v->y, vz = v->z;

    const float tx = 2.0f * (qy * vz - qz * vy);
    const float ty = 2.0f * (qz * vx - qx * vz);
    const float tz = 2.0f * (qx * vy - qy * vx);

    v->x = vx + qw * tx + qy * tz - qz * ty;
    v->y = vy + qw * ty + qz * tx - qx * tz;
    v->z = vz + qw * tz + qx * ty - qy * tx;
}

void quat_to_euler(const quaternion_t *q, float *roll, float *pitch, float *yaw)
{
    const float sinr_cosp = 2.0f * (q->w * q->x + q->y * q->z);
    const float cosr_cosp = 1.0f - 2.0f * (q->x * q->x + q->y * q->y);
    *roll = atan2f(sinr_cosp, cosr_cosp);

    const float sinp = 2.0f * (q->w * q->y - q->z * q->x);
    *pitch = (fabsf(sinp) >= 1.0f) ? copysignf(PI / 2.0f, sinp) : asinf(sinp);

    const float siny_cosp = 2.0f * (q->w * q->z + q->x * q->y);
    const float cosy_cosp = 1.0f - 2.0f * (q->y * q->y + q->z * q->z);
    *yaw = atan2f(siny_cosp, cosy_cosp);
}

void quat_to_gravity(const quaternion_t *q, float *gx, float *gy, float *gz)
{
    *gx = 2.0f * (q->x * q->z - q->w * q->y);
    *gy = 2.0f * (q->w * q->x + q->y * q->z);
    *gz = q->w * q->w - q->x * q->x - q->y * q->y + q->z * q->z;
}

void madgwick_update(quaternion_t *q, float beta, float gx, float gy, float gz,
                     float ax, float ay, float az, float dt)
{
    /* Gradient descent step, unchanged from the RP2350 implementation. Gyro
     * rates are in rad/s. */
    float recipNorm;
    float s0, s1, s2, s3;

    float qDot1 = 0.5f * (-q->x * gx - q->y * gy - q->z * gz);
    float qDot2 = 0.5f * (q->w * gx + q->y * gz - q->z * gy);
    float qDot3 = 0.5f * (q->w * gy - q->x * gz + q->z * gx);
    float qDot4 = 0.5f * (q->w * gz + q->x * gy - q->y * gx);

    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        const float _2q0 = 2.0f * q->w;
        const float _2q1 = 2.0f * q->x;
        const float _2q2 = 2.0f * q->y;
        const float _2q3 = 2.0f * q->z;
        const float _4q0 = 4.0f * q->w;
        const float _4q1 = 4.0f * q->x;
        const float _4q2 = 4.0f * q->y;
        const float _8q1 = 8.0f * q->x;
        const float _8q2 = 8.0f * q->y;
        const float q0q0 = q->w * q->w;
        const float q1q1 = q->x * q->x;
        const float q2q2 = q->y * q->y;
        const float q3q3 = q->z * q->z;

        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q->x - _2q0 * ay - _4q1 +
             _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q->y + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 +
             _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q->z - _2q1 * ax + 4.0f * q2q2 * q->z - _2q2 * ay;

        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    q->w += qDot1 * dt;
    q->x += qDot2 * dt;
    q->y += qDot3 * dt;
    q->z += qDot4 * dt;

    recipNorm = invSqrt(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
    q->w *= recipNorm;
    q->x *= recipNorm;
    q->y *= recipNorm;
    q->z *= recipNorm;
}

void rotate_x(vertex_t *v, float angle)
{
    const float c = cosf(angle);
    const float s = sinf(angle);
    const float y = v->y;
    const float z = v->z;
    v->y = y * c - z * s;
    v->z = y * s + z * c;
}

void rotate_y(vertex_t *v, float angle)
{
    const float c = cosf(angle);
    const float s = sinf(angle);
    const float x = v->x;
    const float z = v->z;
    v->x = x * c + z * s;
    v->z = -x * s + z * c;
}

void rotate_z(vertex_t *v, float angle)
{
    const float c = cosf(angle);
    const float s = sinf(angle);
    const float x = v->x;
    const float y = v->y;
    v->x = x * c - y * s;
    v->y = x * s + y * c;
}

void demo_project(vertex_t v, float distance, float focal, int *x, int *y)
{
    float z = v.z + distance;
    if (z < 0.001f) {
        z = 0.001f;
    }
    *x = DISP_CX + (int)(v.x * focal * DISP_SCALE / z);
    *y = DISP_CY + (int)(v.y * focal * DISP_SCALE / z);
}

static void draw_cube_internal(const vertex_t *vertices, float distance, float focal,
                               bool mono, uint16_t mono_color)
{
    int screen[8][2];
    for (int i = 0; i < 8; i++) {
        demo_project(vertices[i], distance, focal, &screen[i][0], &screen[i][1]);
    }

    for (int i = 0; i < 12; i++) {
        uint16_t color = mono_color;
        if (!mono) {
            /* Back face red, front face green, connecting edges blue. */
            color = (i < 4) ? GFX_RED : (i < 8) ? GFX_GREEN : GFX_BLUE;
        }
        const int v0 = demo_cube_edges[i][0];
        const int v1 = demo_cube_edges[i][1];
        gfx_thick_line(screen[v0][0], screen[v0][1], screen[v1][0], screen[v1][1], 3, color);
    }
}

void demo_draw_cube(const vertex_t *vertices, float distance, float focal)
{
    draw_cube_internal(vertices, distance, focal, false, 0);
}

void demo_draw_cube_mono(const vertex_t *vertices, float distance, float focal, uint16_t color)
{
    draw_cube_internal(vertices, distance, focal, true, color);
}

/* --- IMU ------------------------------------------------------------------ */

/* The sensor core: reads, maps, publishes, and runs whatever filter the current
 * demo installed. Pinned away from the core doing the drawing, so neither the
 * I2C transaction nor the filter maths lands in the frame budget. */

#define SENSOR_CORE        1
#define SENSOR_PERIOD_MS   4        /* 250 Hz, the configured sensor ODR */
#define SENSOR_PRIORITY    5

static portMUX_TYPE s_state_lock = portMUX_INITIALIZER_UNLOCKED;
static imu_sample_t s_sample;
static demo_filter_fn s_filter;
static float s_sensor_rate;
static volatile bool s_sensor_paused;
static volatile bool s_sensor_idle;

void demo_sensor_pause(bool paused)
{
    s_sensor_paused = paused;
    if (!paused) {
        return;
    }
    /* Wait for the task to reach the top of its loop, so no transaction is
     * still in flight when the caller starts probing. */
    for (int i = 0; i < 50 && !s_sensor_idle; i++) {
        demo_delay_ms(2);
    }
}

void demo_lock(void)
{
    portENTER_CRITICAL(&s_state_lock);
}

void demo_unlock(void)
{
    portEXIT_CRITICAL(&s_state_lock);
}

void demo_set_filter(demo_filter_fn filter)
{
    demo_lock();
    s_filter = filter;
    demo_unlock();
}

bool demo_imu_latest(imu_sample_t *out)
{
    bool valid;
    demo_lock();
    if (out != NULL) {
        *out = s_sample;
    }
    valid = s_sample.valid;
    demo_unlock();
    return valid;
}

bool demo_read_imu(vector3f_t *acc, vector3f_t *gyro)
{
    imu_sample_t sample;
    if (!demo_imu_latest(&sample)) {
        return false;
    }
    if (acc != NULL) {
        *acc = sample.acc;
    }
    if (gyro != NULL) {
        *gyro = sample.gyro;
    }
    return true;
}

float demo_sensor_rate(void)
{
    return s_sensor_rate;
}

static void sensor_task(void *arg)
{
    static const int accel_map[3] = BOARD_IMU_ACCEL_MAP;
    static const float accel_sign[3] = BOARD_IMU_ACCEL_SIGN;
    static const int gyro_map[3] = BOARD_IMU_GYRO_MAP;
    static const float gyro_sign[3] = BOARD_IMU_GYRO_SIGN;

    (void)arg;

    TickType_t next_wake = xTaskGetTickCount();
    uint64_t last_us = demo_micros();
    uint64_t window_us = last_us;
    uint32_t window_samples = 0;
    uint32_t seq = 0;

    for (;;) {
        imu_sample_t sample = {0};

        s_sensor_idle = s_sensor_paused;

        if (!s_sensor_paused && qmi8658_read_raw(sample.acc_raw, sample.gyro_raw)) {
            const vector3f_t offset = qmi8658_gyro_offset();
            const float a[3] = {
                sample.acc_raw[0] / QMI8658_ACC_LSB_PER_G,
                sample.acc_raw[1] / QMI8658_ACC_LSB_PER_G,
                sample.acc_raw[2] / QMI8658_ACC_LSB_PER_G,
            };
            const float g[3] = {
                sample.gyro_raw[0] / QMI8658_GYRO_LSB_PER_DPS - offset.x,
                sample.gyro_raw[1] / QMI8658_GYRO_LSB_PER_DPS - offset.y,
                sample.gyro_raw[2] / QMI8658_GYRO_LSB_PER_DPS - offset.z,
            };

            sample.acc.x = a[accel_map[0]] * accel_sign[0];
            sample.acc.y = a[accel_map[1]] * accel_sign[1];
            sample.acc.z = a[accel_map[2]] * accel_sign[2];
            sample.gyro.x = g[gyro_map[0]] * gyro_sign[0];
            sample.gyro.y = g[gyro_map[1]] * gyro_sign[1];
            sample.gyro.z = g[gyro_map[2]] * gyro_sign[2];

            const uint64_t now = demo_micros();
            float dt = (float)(now - last_us) / 1000000.0f;
            last_us = now;
            if (dt > 0.1f) {
                dt = (float)SENSOR_PERIOD_MS / 1000.0f;
            }
            sample.dt = dt;
            sample.seq = ++seq;
            sample.valid = true;

            demo_filter_fn filter;
            demo_lock();
            s_sample = sample;
            filter = s_filter;
            demo_unlock();

            /* Outside the lock: the filter takes its own where it needs it. */
            if (filter != NULL) {
                filter(&sample);
            }

            window_samples++;
            if (now - window_us >= 1000000ULL) {
                s_sensor_rate = (float)window_samples * 1000000.0f /
                                (float)(now - window_us);
                window_samples = 0;
                window_us = now;
            }
        }

        vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(SENSOR_PERIOD_MS));
    }
}

void demo_sensor_start(void)
{
    xTaskCreatePinnedToCore(sensor_task, "imu", 4096, NULL, SENSOR_PRIORITY, NULL,
                            SENSOR_CORE);
}

/* --- launcher services ---------------------------------------------------- */

#define INPUT_RING 32
#define TOUCH_POLL_INTERVAL_US 20000
#define EXIT_HOLD_US 600000
#define EXIT_ZONE_HEIGHT (DISP_H / 6)

static char s_ring[INPUT_RING];
static int s_ring_head;
static int s_ring_tail;
static bool s_exit;
static touch_state_t s_touch;
static uint64_t s_touch_poll_us;
static uint64_t s_touch_hold_us;

static void ring_push(char c)
{
    const int next = (s_ring_head + 1) % INPUT_RING;
    if (next == s_ring_tail) {
        return; /* ring is full, drop the new character */
    }
    s_ring[s_ring_head] = c;
    s_ring_head = next;
}

static void console_poll(void)
{
    uint8_t buf[16];
    int n = 0;

#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    n = usb_serial_jtag_read_bytes(buf, sizeof(buf), 0);
#else
    n = (int)read(STDIN_FILENO, buf, sizeof(buf));
#endif
    if (n <= 0) {
        return;
    }

    for (int i = 0; i < n; i++) {
        const char c = (char)buf[i];
        if (c == 0x1B || c == '~') {
            s_exit = true;
        } else {
            ring_push(c);
        }
    }
}

static void touch_poll(void)
{
    const uint64_t now = demo_micros();
    if (now - s_touch_poll_us < TOUCH_POLL_INTERVAL_US) {
        return;
    }
    s_touch_poll_us = now;

    cst820_read(&s_touch);

    if (s_touch.pressed && s_touch.y < EXIT_ZONE_HEIGHT) {
        if (s_touch_hold_us == 0) {
            s_touch_hold_us = now;
        } else if (now - s_touch_hold_us > EXIT_HOLD_US) {
            s_exit = true;
        }
    } else {
        s_touch_hold_us = 0;
    }
}

bool demo_exit_requested(void)
{
    console_poll();
    touch_poll();
    return s_exit;
}

int demo_read_char(void)
{
    console_poll();
    if (s_ring_tail == s_ring_head) {
        return -1;
    }
    const char c = s_ring[s_ring_tail];
    s_ring_tail = (s_ring_tail + 1) % INPUT_RING;
    return (unsigned char)c;
}

void demo_flush_input(void)
{
    console_poll();
    s_ring_head = 0;
    s_ring_tail = 0;
}

void demo_clear_exit(void)
{
    s_exit = false;
    s_touch_hold_us = 0;
    demo_flush_input();
}

void demo_delay_ms(uint32_t ms)
{
    DEV_Delay_ms(ms);
}

uint32_t demo_millis(void)
{
    return (uint32_t)(DEV_Time_us() / 1000ULL);
}

uint64_t demo_micros(void)
{
    return DEV_Time_us();
}

float demo_delta_seconds(uint64_t *last_us)
{
    const uint64_t now = demo_micros();
    float dt = (float)(now - *last_us) / 1000000.0f;
    *last_us = now;

    if (dt > 0.1f) {
        dt = 0.01f;     /* first pass, or the loop stalled */
    } else if (dt < 0.0005f) {
        dt = 0.0005f;
    }
    return dt;
}

/* Temporary instrumentation: where does a frame actually go? */
static uint64_t s_t_record, s_t_present;
static uint64_t s_mark_begin;
static uint32_t s_stat_frames;

uint16_t *demo_frame_begin(uint16_t clear_color)
{
    gfx_clear(clear_color);
    s_mark_begin = demo_micros();
    return NULL;        /* nothing holds a whole frame */
}

void demo_frame_end(void)
{
    const uint64_t t_recorded = demo_micros();
    s_t_record += t_recorded - s_mark_begin;
    const bool overflowed = gfx_overflowed();

    LCD_2IN1_Present();
    touch_poll();
    s_t_present += demo_micros() - t_recorded;

    if (++s_stat_frames >= 100) {
        printf("\n[frame us] record %llu  wait %llu  total %llu  compose/bounce max %lu us%s\n",
               (unsigned long long)(s_t_record / s_stat_frames),
               (unsigned long long)(s_t_present / s_stat_frames),
               (unsigned long long)((s_t_record + s_t_present) / s_stat_frames),
               (unsigned long)LCD_2IN1_ComposeMaxUs(),
               overflowed ? "  DISPLAY LIST OVERFLOW" : "");
        s_t_record = s_t_present = 0;
        s_stat_frames = 0;
    }
}

bool demo_touch(touch_state_t *out)
{
    touch_poll();
    if (out != NULL) {
        *out = s_touch;
    }
    return s_touch.pressed;
}

int demo_safe_left(int y, int height)
{
    /* The tighter of the box's two edges decides how much room there is. */
    int half = gfx_visible_half_width(y);
    const int other = gfx_visible_half_width(y + height);
    if (other < half) {
        half = other;
    }
    half -= 6;                  /* keep clear of the bezel */
    if (half < 0) {
        half = 0;
    }
    return DISP_CX - half;
}

int demo_safe_right(int y, int height)
{
    return DISP_W - demo_safe_left(y, height);
}

void demo_draw_exit_hint(void)
{
    /* Kept short and lifted clear of the bottom arc so it stays inside the
     * visible circle. */
    gfx_text_centered(DISP_CX, 430, "hold top or press ESC to exit", GFX_DGREY, 1);
}

void demo_show_message(const char *title, const char *detail, uint16_t color)
{
    demo_frame_begin(GFX_BLACK);
    gfx_text_centered(DISP_CX, DISP_CY - 20, title, color, 3);
    if (detail != NULL) {
        gfx_text_centered(DISP_CX, DISP_CY + 30, detail, GFX_GREY, 2);
    }
    demo_frame_end();
}
