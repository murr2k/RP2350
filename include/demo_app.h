#ifndef DEMO_APP_H
#define DEMO_APP_H

#include "system_types.h"

/**
 * @brief Initialize the demo application
 * @return SYSTEM_OK on success, error code on failure
 */
system_result_t demo_app_init(void);

/**
 * @brief Update the demo application logic (call from main loop)
 * @return SYSTEM_OK on success, error code on failure
 */
system_result_t demo_app_update(void);

/**
 * @brief Update the display (call from display thread)
 */
void demo_app_update_display(void);

#endif // DEMO_APP_H