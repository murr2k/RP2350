#ifndef SYSTEM_TYPES_H
#define SYSTEM_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

// System error codes
typedef enum {
    SYSTEM_OK = 0,
    SYSTEM_ERROR = -1,
    SYSTEM_ERROR_INVALID_PARAM = -2,
    SYSTEM_ERROR_NOT_INITIALIZED = -3,
    SYSTEM_ERROR_TIMEOUT = -4,
    SYSTEM_ERROR_NO_MEMORY = -5,
    SYSTEM_ERROR_BUSY = -6,
    SYSTEM_ERROR_NOT_SUPPORTED = -7,
    SYSTEM_ERROR_IO = -8,
    SYSTEM_ERROR_CRC = -9,
    SYSTEM_ERROR_OVERFLOW = -10
} system_result_t;

// Color type for display
typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} rgb_color_t;

typedef uint16_t color565_t;  // RGB565 format

// 2D Point and Rectangle structures
typedef struct {
    int16_t x;
    int16_t y;
} point_t;

typedef struct {
    int16_t x;
    int16_t y;
    uint16_t width;
    uint16_t height;
} rect_t;

// IMU data structures
typedef struct {
    float x;
    float y;
    float z;
} vector3_t;

typedef struct {
    vector3_t accelerometer;    // m/s²
    vector3_t gyroscope;       // rad/s
    float temperature;         // °C
    uint64_t timestamp_us;     // microseconds
    bool valid;
} imu_data_t;

// Button states
typedef enum {
    BUTTON_RELEASED = 0,
    BUTTON_PRESSED = 1,
    BUTTON_HELD = 2
} button_state_t;

typedef struct {
    button_state_t a;
    button_state_t b;
    button_state_t x;
    button_state_t y;
    uint32_t press_time_ms[4];  // Time each button was pressed
} button_states_t;

// System state
typedef struct {
    bool initialized;
    uint32_t uptime_ms;
    float cpu_temperature;
    uint16_t vbat_mv;
    uint8_t brightness;
    bool usb_connected;
} system_state_t;

// Display buffer
typedef struct {
    color565_t *buffer;
    uint16_t width;
    uint16_t height;
    bool dirty;
} framebuffer_t;

// Event system
typedef enum {
    EVENT_NONE = 0,
    EVENT_BUTTON_PRESS,
    EVENT_BUTTON_RELEASE,
    EVENT_IMU_DATA,
    EVENT_TIMER,
    EVENT_USB_CONNECT,
    EVENT_USB_DISCONNECT,
    EVENT_DISPLAY_UPDATE
} event_type_t;

typedef struct {
    event_type_t type;
    uint32_t timestamp_ms;
    union {
        struct {
            uint8_t button_id;
            button_state_t state;
        } button;
        imu_data_t imu;
        uint32_t timer_id;
    } data;
} system_event_t;

// Configuration structure
typedef struct {
    uint8_t display_brightness;
    uint16_t imu_sample_rate_hz;
    bool auto_sleep_enabled;
    uint32_t auto_sleep_timeout_ms;
    bool debug_output_enabled;
} system_config_t;

// Utility macros
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define CLAMP(value, min_val, max_val) (MIN(MAX((value), (min_val)), (max_val)))

// RGB565 color conversion macros
#define RGB_TO_RGB565(r, g, b) (((uint16_t)(r & 0xF8) << 8) | ((uint16_t)(g & 0xFC) << 3) | ((uint16_t)(b & 0xF8) >> 3))
#define RGB565_TO_R(color) ((uint8_t)(((color) >> 8) & 0xF8))
#define RGB565_TO_G(color) ((uint8_t)(((color) >> 3) & 0xFC))
#define RGB565_TO_B(color) ((uint8_t)(((color) << 3) & 0xF8))

// Common colors
#define COLOR_BLACK     0x0000
#define COLOR_WHITE     0xFFFF
#define COLOR_RED       0xF800
#define COLOR_GREEN     0x07E0
#define COLOR_BLUE      0x001F
#define COLOR_CYAN      0x07FF
#define COLOR_MAGENTA   0xF81F
#define COLOR_YELLOW    0xFFE0
#define COLOR_ORANGE    0xFC00
#define COLOR_PURPLE    0x8010
#define COLOR_GRAY      0x8410
#define COLOR_DARK_GRAY 0x4208

#endif // SYSTEM_TYPES_H