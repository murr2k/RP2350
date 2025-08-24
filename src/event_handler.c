#include "event_handler.h"
#include "system_types.h"
#include "board_config.h"
#include "button_driver.h"
#include "imu_driver.h"
#include "adc_driver.h"

#include <stdio.h>
#include <string.h>

// Event queue
#define EVENT_QUEUE_SIZE 32
static system_event_t event_queue[EVENT_QUEUE_SIZE];
static uint8_t queue_head = 0;
static uint8_t queue_tail = 0;
static uint8_t queue_count = 0;

// Event callbacks
static void (*button_event_callback)(button_id_t button, button_event_t event) = NULL;
static void (*imu_event_callback)(imu_data_t *data) = NULL;

static bool queue_is_full(void) {
    return queue_count >= EVENT_QUEUE_SIZE;
}

static bool queue_is_empty(void) {
    return queue_count == 0;
}

static system_result_t queue_push(const system_event_t *event) {
    if (queue_is_full()) {
        DEBUG_PRINT("Event queue overflow!");
        return SYSTEM_ERROR_OVERFLOW;
    }
    
    event_queue[queue_head] = *event;
    queue_head = (queue_head + 1) % EVENT_QUEUE_SIZE;
    queue_count++;
    
    return SYSTEM_OK;
}

static system_result_t queue_pop(system_event_t *event) {
    if (queue_is_empty()) {
        return SYSTEM_ERROR_NO_MEMORY;
    }
    
    *event = event_queue[queue_tail];
    queue_tail = (queue_tail + 1) % EVENT_QUEUE_SIZE;
    queue_count--;
    
    return SYSTEM_OK;
}

static void button_callback(button_id_t button, button_event_t event) {
    system_event_t sys_event = {0};
    sys_event.type = (event == BUTTON_EVENT_PRESS) ? EVENT_BUTTON_PRESS : EVENT_BUTTON_RELEASE;
    sys_event.timestamp_ms = to_ms_since_boot(get_absolute_time());
    sys_event.data.button.button_id = button;
    sys_event.data.button.state = (event == BUTTON_EVENT_PRESS) ? BUTTON_PRESSED : BUTTON_RELEASED;
    
    if (queue_push(&sys_event) != SYSTEM_OK) {
        DEBUG_PRINT("Failed to queue button event");
    } else {
        DEBUG_PRINT("Button %s %s", button_get_name(button), 
                   (event == BUTTON_EVENT_PRESS) ? "pressed" : "released");
    }
    
    // Call user callback if registered
    if (button_event_callback) {
        button_event_callback(button, event);
    }
}

static void imu_data_ready_callback(void) {
    system_event_t sys_event = {0};
    sys_event.type = EVENT_IMU_DATA;
    sys_event.timestamp_ms = to_ms_since_boot(get_absolute_time());
    
    // Read IMU data
    if (imu_read_data(&sys_event.data.imu) == SYSTEM_OK) {
        if (queue_push(&sys_event) != SYSTEM_OK) {
            DEBUG_PRINT("Failed to queue IMU event");
        }
        
        // Call user callback if registered
        if (imu_event_callback) {
            imu_event_callback(&sys_event.data.imu);
        }
    }
}

system_result_t event_handler_init(void) {
    // Clear event queue
    memset(event_queue, 0, sizeof(event_queue));
    queue_head = 0;
    queue_tail = 0;
    queue_count = 0;
    
    // Set up button event callback
    system_result_t result = button_set_callback(button_callback);
    if (result != SYSTEM_OK) {
        printf("Failed to set button callback: %d\n", result);
        return result;
    }
    
    // Set up IMU data ready callback
    result = imu_set_data_ready_callback(imu_data_ready_callback);
    if (result != SYSTEM_OK) {
        printf("Failed to set IMU callback: %d\n", result);
        return result;
    }
    
    printf("Event handler initialized\n");
    return SYSTEM_OK;
}

system_result_t event_handler_process(void) {
    system_event_t event;
    uint8_t processed_count = 0;
    
    // Process all events in queue (with limit to prevent infinite loop)
    while (!queue_is_empty() && processed_count < EVENT_QUEUE_SIZE) {
        if (queue_pop(&event) == SYSTEM_OK) {
            switch (event.type) {
                case EVENT_BUTTON_PRESS:
                    DEBUG_PRINT("Processing button press event: Button %d", 
                               event.data.button.button_id);
                    break;
                    
                case EVENT_BUTTON_RELEASE:
                    DEBUG_PRINT("Processing button release event: Button %d", 
                               event.data.button.button_id);
                    break;
                    
                case EVENT_IMU_DATA:
                    DEBUG_PRINT("Processing IMU data event: Accel(%.2f,%.2f,%.2f)", 
                               event.data.imu.accelerometer.x,
                               event.data.imu.accelerometer.y,
                               event.data.imu.accelerometer.z);
                    break;
                    
                case EVENT_TIMER:
                    DEBUG_PRINT("Processing timer event: Timer %lu", event.data.timer_id);
                    break;
                    
                case EVENT_USB_CONNECT:
                    DEBUG_PRINT("Processing USB connect event");
                    break;
                    
                case EVENT_USB_DISCONNECT:
                    DEBUG_PRINT("Processing USB disconnect event");
                    break;
                    
                case EVENT_DISPLAY_UPDATE:
                    DEBUG_PRINT("Processing display update event");
                    break;
                    
                default:
                    DEBUG_PRINT("Unknown event type: %d", event.type);
                    break;
            }
            
            processed_count++;
        }
    }
    
    return SYSTEM_OK;
}

system_result_t event_handler_post_event(const system_event_t *event) {
    if (!event) {
        return SYSTEM_ERROR_INVALID_PARAM;
    }
    
    return queue_push(event);
}

system_result_t event_handler_set_button_callback(void (*callback)(button_id_t button, button_event_t event)) {
    button_event_callback = callback;
    return SYSTEM_OK;
}

system_result_t event_handler_set_imu_callback(void (*callback)(imu_data_t *data)) {
    imu_event_callback = callback;
    return SYSTEM_OK;
}

uint8_t event_handler_get_queue_count(void) {
    return queue_count;
}

void event_handler_clear_queue(void) {
    queue_head = 0;
    queue_tail = 0;
    queue_count = 0;
    DEBUG_PRINT("Event queue cleared");
}