/**
 * Fetch the time of day over the network and put it in the RTC.
 *
 * Runs once, at boot, on its own task, and then takes the radio back down. The
 * panel has no frame buffer and composes each row inside an interrupt against a
 * deadline set by the pixel clock, and the WiFi stack executes from the same
 * PSRAM the composer reads its display list from. Leaving it associated would
 * put an unpredictable second consumer on that bandwidth for the sake of a
 * reading taken once. So: connect, ask, set the clock, stop.
 *
 * There is no backup cell on the clock yet, so this is the only thing that
 * knows what time it is. When one is fitted the clock survives a power cycle
 * and this becomes a correction rather than the sole source, which needs no
 * change here.
 */

#ifndef NET_TIME_H
#define NET_TIME_H

#include <stdbool.h>

typedef enum {
    NET_TIME_IDLE = 0,      /**< not started */
    NET_TIME_NO_CONFIG,     /**< no wifi_secrets.h, so nothing to connect to */
    NET_TIME_CONNECTING,
    NET_TIME_SYNCING,       /**< associated, waiting on the time server */
    NET_TIME_SET,           /**< the clock is right and the radio is off again */
    NET_TIME_FAILED,
} net_time_state_t;

/** Start the one shot fetch in the background. Returns immediately: the picker
 *  is up and usable throughout. */
void net_time_start(void);

net_time_state_t net_time_state(void);

/** Short word for the status line, never NULL. */
const char *net_time_status(void);

#endif /* NET_TIME_H */
