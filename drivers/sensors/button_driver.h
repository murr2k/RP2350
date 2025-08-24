#ifndef BUTTON_DRIVER_H
#define BUTTON_DRIVER_H

#include "system_types.h"
#include "board_config.h"

// Button IDs
typedef enum {
    BUTTON_A = 0,
    BUTTON_B = 1,
    BUTTON_X = 2,
    BUTTON_Y = 3,
    BUTTON_COUNT = 4
} button_id_t;

// Button event types
typedef enum {
    BUTTON_EVENT_NONE = 0,
    BUTTON_EVENT_PRESS,
    BUTTON_EVENT_RELEASE,
    BUTTON_EVENT_HOLD,
    BUTTON_EVENT_REPEAT
} button_event_t;

// Button configuration
typedef struct {
    uint32_t debounce_time_ms;     // Debounce time in milliseconds
    uint32_t hold_time_ms;         // Time to trigger hold event
    uint32_t repeat_time_ms;       // Time between repeat events
    bool enable_repeat;            // Enable repeat events when held
} button_config_t;

/**
 * @brief Initialize the button driver
 * @return SYSTEM_OK on success, error code on failure
 */
system_result_t button_init(void);

/**
 * @brief Configure button behavior
 * @param config Button configuration structure
 * @return SYSTEM_OK on success, error code on failure
 */
system_result_t button_configure(const button_config_t *config);

/**
 * @brief Update button states (call this regularly from main loop)
 * @return SYSTEM_OK on success, error code on failure
 */
system_result_t button_update(void);

/**
 * @brief Get current state of a button
 * @param button_id Button to check
 * @return Button state
 */
button_state_t button_get_state(button_id_t button_id);

/**
 * @brief Get all button states
 * @param states Pointer to button states structure
 * @return SYSTEM_OK on success, error code on failure
 */
system_result_t button_get_all_states(button_states_t *states);

/**
 * @brief Check if a button was just pressed
 * @param button_id Button to check
 * @return true if button was just pressed, false otherwise
 */
bool button_was_pressed(button_id_t button_id);

/**
 * @brief Check if a button was just released
 * @param button_id Button to check
 * @return true if button was just released, false otherwise
 */
bool button_was_released(button_id_t button_id);

/**
 * @brief Check if a button is currently held
 * @param button_id Button to check
 * @return true if button is held, false otherwise
 */
bool button_is_held(button_id_t button_id);

/**
 * @brief Get how long a button has been pressed
 * @param button_id Button to check
 * @return Press duration in milliseconds, 0 if not pressed
 */
uint32_t button_get_press_duration(button_id_t button_id);

/**
 * @brief Set callback function for button events
 * @param callback Function to call when button events occur
 * @return SYSTEM_OK on success, error code on failure
 */
system_result_t button_set_callback(void (*callback)(button_id_t button, button_event_t event));

/**
 * @brief Clear button event flags (call after handling events)
 * @param button_id Button to clear, or BUTTON_COUNT to clear all
 */
void button_clear_events(button_id_t button_id);

/**
 * @brief Get button name string
 * @param button_id Button ID
 * @return Button name string
 */
const char* button_get_name(button_id_t button_id);

#endif // BUTTON_DRIVER_H