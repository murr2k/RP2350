#include "rtc_pcf85063.h"

#include "board_config.h"
#include "dev_config.h"
#include "esp_log.h"

static const char *TAG = "rtc";

/* Register map. Time starts at 0x04 and the seven bytes are contiguous, so a
 * whole reading is one transfer and cannot be torn across a minute rollover. */
#define REG_CONTROL_1   0x00
#define REG_SECONDS     0x04

#define CTRL1_STOP      0x20    /* halts the counters so a write lands cleanly */
#define CTRL1_12_24     0x02    /* 0 keeps it in 24 hour mode */
#define SECONDS_OS      0x80    /* oscillator stopped: the time is not to be trusted */

static bool s_present;

static uint8_t from_bcd(uint8_t v)
{
    return (uint8_t)((v >> 4) * 10 + (v & 0x0F));
}

static uint8_t to_bcd(uint8_t v)
{
    return (uint8_t)(((v / 10) << 4) | (v % 10));
}

bool pcf85063_init(void)
{
    if (!DEV_I2C_Probe(BOARD_RTC_ADDR)) {
        ESP_LOGW(TAG, "no clock at 0x%02x", BOARD_RTC_ADDR);
        s_present = false;
        return false;
    }

    /* Clear everything except the mode bit: 24 hour, running, no interrupts. */
    if (DEV_I2C_Write_Byte(BOARD_RTC_ADDR, REG_CONTROL_1, 0x00) != ESP_OK) {
        s_present = false;
        return false;
    }
    s_present = true;

    pcf85063_time_t now;
    if (pcf85063_get(&now)) {
        ESP_LOGI(TAG, "ready at 0x%02x, reads %04u-%02u-%02u %02u:%02u:%02u%s",
                 BOARD_RTC_ADDR, now.year, now.month, now.day,
                 now.hour, now.minute, now.second,
                 pcf85063_running() ? "" : " (oscillator stopped, time not set)");
    }
    return true;
}

bool pcf85063_present(void)
{
    return s_present;
}

bool pcf85063_running(void)
{
    if (!s_present) {
        return false;
    }
    uint8_t seconds = 0;
    if (DEV_I2C_Read_nByte(BOARD_RTC_ADDR, REG_SECONDS, &seconds, 1) != ESP_OK) {
        return false;
    }
    return (seconds & SECONDS_OS) == 0;
}

bool pcf85063_get(pcf85063_time_t *out)
{
    if (!s_present || out == NULL) {
        return false;
    }

    uint8_t buf[7] = {0};
    if (DEV_I2C_Read_nByte(BOARD_RTC_ADDR, REG_SECONDS, buf, sizeof(buf)) != ESP_OK) {
        return false;
    }

    out->second  = from_bcd(buf[0] & 0x7F);
    out->minute  = from_bcd(buf[1] & 0x7F);
    out->hour    = from_bcd(buf[2] & 0x3F);
    out->day     = from_bcd(buf[3] & 0x3F);
    out->weekday = (uint8_t)(buf[4] & 0x07);
    out->month   = from_bcd(buf[5] & 0x1F);
    out->year    = (uint16_t)(2000 + from_bcd(buf[6]));
    return true;
}

bool pcf85063_set(const pcf85063_time_t *in)
{
    if (!s_present || in == NULL) {
        return false;
    }

    /* Stop the counters first. Without it a carry can land between the write of
     * one field and the next, which is how a clock ends up an hour out once in
     * every few thousand settings. */
    if (DEV_I2C_Write_Byte(BOARD_RTC_ADDR, REG_CONTROL_1, CTRL1_STOP) != ESP_OK) {
        return false;
    }

    /* Seconds are written with the top bit clear, which is what clears the
     * oscillator stop flag and marks the time as trustworthy again. */
    const uint8_t payload[8] = {
        REG_SECONDS,
        to_bcd(in->second) & 0x7F,
        to_bcd(in->minute),
        to_bcd(in->hour),
        to_bcd(in->day),
        (uint8_t)(in->weekday & 0x07),
        to_bcd(in->month),
        to_bcd((uint8_t)(in->year % 100)),
    };
    const esp_err_t err = DEV_I2C_Write_nByte(BOARD_RTC_ADDR, payload, sizeof(payload));

    /* Start it again whatever happened, so a failed write does not leave a
     * stopped clock behind. */
    DEV_I2C_Write_Byte(BOARD_RTC_ADDR, REG_CONTROL_1, 0x00);
    return err == ESP_OK;
}

bool pcf85063_get_tm(struct tm *out)
{
    pcf85063_time_t now;
    if (out == NULL || !pcf85063_get(&now)) {
        return false;
    }

    out->tm_year = (int)now.year - 1900;
    out->tm_mon = (int)now.month - 1;
    out->tm_mday = now.day;
    out->tm_hour = now.hour;
    out->tm_min = now.minute;
    out->tm_sec = now.second;
    out->tm_wday = now.weekday;
    out->tm_isdst = -1;
    return true;
}

bool pcf85063_set_tm(const struct tm *in)
{
    if (in == NULL) {
        return false;
    }
    const pcf85063_time_t now = {
        .year = (uint16_t)(in->tm_year + 1900),
        .month = (uint8_t)(in->tm_mon + 1),
        .day = (uint8_t)in->tm_mday,
        .hour = (uint8_t)in->tm_hour,
        .minute = (uint8_t)in->tm_min,
        .second = (uint8_t)in->tm_sec,
        .weekday = (uint8_t)in->tm_wday,
    };
    return pcf85063_set(&now);
}
