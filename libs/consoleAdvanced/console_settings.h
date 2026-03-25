#ifndef CONSOLE_SETTINGS_H
#define CONSOLE_SETTINGS_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the console peripheral (UART/USB-CDC)
 */
void initialize_console_peripheral(void);

/**
 * @brief Initialize the console library with linenoise
 * 
 * @param history_path Path to history file (can be NULL)
 */
void initialize_console_library(const char *history_path);

/**
 * @brief Set up the console prompt with optional color
 * 
 * @param prompt_str Base prompt string
 * @return Allocated prompt string (caller must free)
 */
char *setup_prompt(const char *prompt_str);

#ifdef __cplusplus
}
#endif

#endif // CONSOLE_SETTINGS_H