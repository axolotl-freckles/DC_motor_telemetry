#ifndef CONSOLE_ADVANCED_HPP
#define CONSOLE_ADVANCED_HPP

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"

/**
 * @brief Initialize and start the advanced console
 * 
 * This function sets up:
 * - UART/USB-CDC console peripheral
 * - Console library with linenoise editing
 * - Help command
 * - System commands (version, restart, free, heap, tasks, log_level)
 * - WiFi commands (join, scan, disconnect)
 * - NVS commands (set, get, erase, list)
 */
esp_err_t console_advanced_init(void);

/**
 * @brief Start the main console loop
 * 
 * This function blocks indefinitely, processing console input.
 * It should be called from app_main or a separate FreeRTOS task.
 */
void console_advanced_start(void);

#ifdef __cplusplus
}
#endif

#endif // CONSOLE_ADVANCED_HPP