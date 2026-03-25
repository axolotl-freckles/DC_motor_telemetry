# Console Advanced Component

A complete console implementation for ESP32 based on the ESP-IDF advanced console example. Provides interactive command-line interface with multiple command groups.

## Features

- **UART/USB-CDC Console**: Configurable serial interface
- **Linenoise Integration**: Line editing, history, and command completion
- **System Commands**:
  - `version`: Display chip info and IDF version
  - `restart`: Software reset
  - `free`: Show heap memory statistics
  - `tasks`: List all FreeRTOS tasks
  - `log_level`: Set ESP_LOG level per tag
  - `deep_sleep`: Enter deep sleep mode (if supported)
  - `light_sleep`: Enter light sleep mode (if supported)

- **WiFi Commands**:
  - `join <ssid> [<pass>]`: Connect to WiFi network
  - `scan`: Scan available networks (expandable)

- **NVS Commands**:
  - `nvs_init`: Initialize NVS flash
  - `nvs_set <key> <type> -v <value>`: Store value in NVS
  - `nvs_get <key> <type>`: Retrieve value from NVS
  - `nvs_erase <key>`: Delete key from NVS

Supported NVS types: `u8`, `u16`, `u32`, `u64`, `i8`, `i16`, `i32`, `i64`, `str`

## Usage

### Initialization

```cpp
#include "console_advanced.hpp"

extern "C" void app_main(void)
{
    // Initialize the console
    console_advanced_init();
    
    // Start the main console loop (blocks indefinitely)
    console_advanced_start();
}
```

### CMakeLists.txt

```cmake
idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES "consoleAdvanced"
)
```

## Components

### Files

- `console_advanced.hpp/cpp`: Main C++ wrapper and entry points
- `console_settings.h/c`: UART and linenoise initialization
- `cmd_system_common.c`: System information and control commands
- `cmd_system_sleep.c`: Sleep mode commands
- `cmd_wifi.c`: WiFi connectivity commands
- `cmd_nvs.c`: Non-volatile storage commands

### Architecture

The component follows a modular design:

1. **Console Peripheral**: Initializes UART/USB-CDC
2. **Console Library**: Sets up ESP-IDF console framework
3. **Command Registration**: Each module registers its commands independently
4. **Main Loop**: Linenoise-based interactive shell

## Extending

To add custom commands:

1. Create a new `.c` file with your command functions
2. Implement a `register_*()` function to register commands
3. Add the file to `CMakeLists.txt`
4. Call the register function in `console_advanced.cpp`

### Example Custom Command

```c
static int my_command(int argc, char **argv)
{
    printf("Hello from custom command!\r\n");
    return 0;
}

void register_my_commands(void)
{
    const esp_console_cmd_t cmd = {
        .command = "mycommand",
        .help = "My custom command",
        .hint = NULL,
        .func = &my_command,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}
```

Then add to `console_advanced.cpp`:
```cpp
extern "C" {
    extern void register_my_commands(void);
}

esp_err_t console_advanced_init(void)
{
    // ... existing code ...
    register_my_commands();
    return ESP_OK;
}
```

## Dependencies

- `console`: ESP-IDF console library
- `esp_driver_uart`: UART driver
- `esp_driver_gpio`: GPIO driver (for sleep wakeup)
- `nvs_flash`: Non-volatile storage
- `freertos`: Real-time OS

## Notes

- The component uses 115200 baud rate by default
- Console buffer size: 256 characters
- Command history: 100 lines
- Terminal detection: Automatic (dumb terminal fallback available)
