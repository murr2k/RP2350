#ifndef EVENT_HANDLER_H
#define EVENT_HANDLER_H

#include "system_types.h"
#include "button_driver.h"

/**
 * @brief Initialize the event handler system
 * @return SYSTEM_OK on success, error code on failure
 */
system_result_t event_handler_init(void);

/**
 * @brief Process all queued events (call from main loop)
 * @return SYSTEM_OK on success, error code on failure
 */
system_result_t event_handler_process(void);

/**
 * @brief Post an event to the event queue
 * @param event Pointer to event structure
 * @return SYSTEM_OK on success, error code on failure
 */
system_result_t event_handler_post_event(const system_event_t *event);

/**
 * @brief Set button event callback
 * @param callback Function to call for button events
 * @return SYSTEM_OK on success, error code on failure
 */
system_result_t event_handler_set_button_callback(void (*callback)(button_id_t button, button_event_t event));

/**
 * @brief Set IMU data callback
 * @param callback Function to call for IMU data events
 * @return SYSTEM_OK on success, error code on failure
 */
system_result_t event_handler_set_imu_callback(void (*callback)(imu_data_t *data));

/**
 * @brief Get number of events in queue
 * @return Number of events in queue
 */
uint8_t event_handler_get_queue_count(void);

/**
 * @brief Clear all events from queue
 */
void event_handler_clear_queue(void);

#endif // EVENT_HANDLER_H