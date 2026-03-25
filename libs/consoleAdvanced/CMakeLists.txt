idf_component_register(
    SRCS "console_advanced.cpp" "console_settings.c" 
         "cmd_system_common.c" "cmd_system_sleep.c" 
         "cmd_wifi.c" "cmd_nvs.c"
    INCLUDE_DIRS "."
    REQUIRES "console" "esp_driver_uart" "esp_driver_gpio" 
             "nvs_flash" "esp_wifi" "freertos" "esp_event" "esp_netif"
)