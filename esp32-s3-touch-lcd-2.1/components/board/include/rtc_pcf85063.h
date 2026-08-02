/**
 * PCF85063 real time clock.
 *
 * The RP2350 board had no clock of its own, so this driver has no counterpart
 * in the original tree. The part sits on the same I2C bus as everything else.
 *
 * There is no backup cell fitted yet. The chip has the pin for one and keeps
 * running off it when the board is unpowered, but as things stand the clock
 * loses the time whenever the power goes, and says so through pcf85063_running():
 * its oscillator stop flag comes up set after every cold start. That is why the
 * time is fetched over the network at boot rather than trusted from the part.
 * Fit the cell and nothing here needs changing, the flag simply stops coming up
 * set and the network fetch becomes a correction rather than the only source.
 */

#ifndef RTC_PCF85063_H
#define RTC_PCF85063_H

#include <stdbool.h>
#include <stdint.h>
#include <time.h>

typedef struct {
    uint16_t year;      /**< full year, 2000 to 2099 */
    uint8_t month;      /**< 1 to 12 */
    uint8_t day;        /**< 1 to 31 */
    uint8_t hour;       /**< 0 to 23, the part is held in 24 hour mode */
    uint8_t minute;
    uint8_t second;
    uint8_t weekday;    /**< 0 = Sunday */
} pcf85063_time_t;

/** Check the part is there and put it in 24 hour mode. Call once at boot,
 *  before anything else starts using the bus. */
bool pcf85063_init(void);

bool pcf85063_present(void);

/** False if the oscillator has stopped since the time was last set, which
 *  means whatever it reads back is meaningless. Set after every cold start
 *  until a backup cell is fitted. */
bool pcf85063_running(void);

bool pcf85063_get(pcf85063_time_t *out);
bool pcf85063_set(const pcf85063_time_t *in);

/** Same, in the terms the C library uses. Fields outside the part's range are
 *  ignored on the way in and filled in on the way out. */
bool pcf85063_get_tm(struct tm *out);
bool pcf85063_set_tm(const struct tm *in);

#endif /* RTC_PCF85063_H */
