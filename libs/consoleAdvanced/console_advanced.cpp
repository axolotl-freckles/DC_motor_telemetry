#include <stdio.h>
#include <string.h>
#include "esp_system.h"
#include "esp_log.h"
#include "esp_console.h"
#include "linenoise/linenoise.h"
#include "argtable3/argtable3.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "console_settings.h"
#include "console_advanced.hpp"

extern "C" {
    extern void register_system_common(void);
    extern void register_system_deep_sleep(void);
    extern void register_system_light_sleep(void);
    extern void register_wifi(void);
    extern void register_nvs(void);
}

static const char* TAG = "console_advanced";

#define HISTORY_PATH NULL

static void initialize_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

esp_err_t console_advanced_init(void)
{
    ESP_LOGI(TAG, "Initializing Advanced Console...");
    
    initialize_nvs();
    
    initialize_console_peripheral();
    
    initialize_console_library(HISTORY_PATH);
    
    esp_console_register_help_command();
    register_system_common();

#if SOC_LIGHT_SLEEP_SUPPORTED
    register_system_light_sleep();
#endif

#if SOC_DEEP_SLEEP_SUPPORTED
    register_system_deep_sleep();
#endif

    register_wifi();
    register_nvs();
    
    ESP_LOGI(TAG, "Console initialized successfully");
    return ESP_OK;
}

void console_advanced_start(void)
{
    const char *prompt = setup_prompt("esp32>");
    
    printf("\n"
           "This is an advanced console example.\n"
           "Type 'help' to get the list of commands.\n"
           "Use UP/DOWN arrows to navigate through command history.\n"
           "Press TAB when typing command name to auto-complete.\n"
           "Ctrl+C will terminate the console environment.\n\n");

    while(true) {
        char* line = linenoise(prompt);

        if (line == NULL) {
            break;
        }

        if (strlen(line) > 0) {
            linenoiseHistoryAdd(line);
        }

        int ret;
        esp_err_t err = esp_console_run(line, &ret);
        if (err == ESP_ERR_NOT_FOUND) {
            printf("Unrecognized command\n");
        } else if (err == ESP_ERR_INVALID_ARG) {
            // command was empty
        } else if (err == ESP_OK && ret != ESP_OK) {
            printf("Command returned non-zero error code: 0x%x (%s)\n", ret, esp_err_to_name(ret));
        } else if (err != ESP_OK) {
            printf("Internal error: %s\n", esp_err_to_name(err));
        }
        linenoiseFree(line);
    }

    ESP_LOGE(TAG, "Error or end-of-input, terminating console");
    esp_console_deinit();
}
