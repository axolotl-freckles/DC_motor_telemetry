#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <inttypes.h>
#include "esp_log.h"
#include "esp_console.h"
#include "esp_chip_info.h"
#include "argtable3/argtable3.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cmd_system.h"
#include "sdkconfig.h"

static const char *TAG = "cmd_system";

static int get_version(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    esp_chip_info_t info;
    esp_chip_info(&info);
    
    printf("IDF Version: %s\r\n", esp_get_idf_version());
    printf("Chip Info:\r\n");
    printf("\tCores: %d\r\n", info.cores);
    printf("\tRevision: %d\r\n", info.revision);
    
    return 0;
}

static int restart(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    ESP_LOGI(TAG, "Restarting...");
    esp_restart();
    return 0;
}

static int get_free_mem(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    printf("Free heap size: %ld bytes\r\n", esp_get_free_heap_size());
    printf("Minimum ever free: %ld bytes\r\n", esp_get_minimum_free_heap_size());
    return 0;
}

static int get_tasks_info(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    
    int task_count = uxTaskGetNumberOfTasks();
    printf("Number of tasks: %d\r\n", task_count);
    
    return 0;
}

static struct {
    struct arg_str *tag;
    struct arg_str *level;
    struct arg_end *end;
} log_level_args;

static int set_log_level(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&log_level_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, log_level_args.end, argv[0]);
        return 1;
    }
    
    const char *tag = "*";
    if (log_level_args.tag->count > 0) {
        tag = log_level_args.tag->sval[0];
    }
    
    const char *level_str = log_level_args.level->sval[0];
    esp_log_level_t level = ESP_LOG_INFO;
    
    if (strcasecmp(level_str, "none") == 0) level = ESP_LOG_NONE;
    else if (strcasecmp(level_str, "error") == 0) level = ESP_LOG_ERROR;
    else if (strcasecmp(level_str, "warn") == 0) level = ESP_LOG_WARN;
    else if (strcasecmp(level_str, "info") == 0) level = ESP_LOG_INFO;
    else if (strcasecmp(level_str, "debug") == 0) level = ESP_LOG_DEBUG;
    else if (strcasecmp(level_str, "verbose") == 0) level = ESP_LOG_VERBOSE;
    else {
        printf("Invalid log level\r\n");
        return 1;
    }
    
    esp_log_level_set(tag, level);
    printf("Log level for %s set to %s\r\n", tag, level_str);
    
    return 0;
}

void register_system_common(void)
{
    const esp_console_cmd_t version_cmd = {
        .command = "version",
        .help = "Get version of ESP-IDF and chip info",
        .hint = NULL,
        .func = &get_version,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&version_cmd));

    const esp_console_cmd_t restart_cmd = {
        .command = "restart",
        .help = "Soft reset of the CPU",
        .hint = NULL,
        .func = &restart,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&restart_cmd));

    const esp_console_cmd_t free_mem_cmd = {
        .command = "free",
        .help = "Get the current size of free heap memory",
        .hint = NULL,
        .func = &get_free_mem,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&free_mem_cmd));

    const esp_console_cmd_t tasks_cmd = {
        .command = "tasks",
        .help = "Get information about running tasks",
        .hint = NULL,
        .func = &get_tasks_info,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&tasks_cmd));

    log_level_args.tag = arg_str0("t", "tag", "<tag>", "Tag of the log, '*' for all");
    log_level_args.level = arg_str1(NULL, NULL, "<level>", "Log level (none|error|warn|info|debug|verbose)");
    log_level_args.end = arg_end(2);

    const esp_console_cmd_t log_level_cmd = {
        .command = "log_level",
        .help = "Set log level for a tag",
        .hint = NULL,
        .func = &set_log_level,
        .argtable = &log_level_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&log_level_cmd));
}
