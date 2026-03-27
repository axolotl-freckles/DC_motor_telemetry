#include <stdio.h>

#include "esp_log.h"

#include "freertos/FreeRTOS.h"

#include "controller.hpp"
#include "controllers/pid_controller.hpp"
#include "filters.hpp"

#include "tasks.hpp"
#include "controller_task.hpp"

extern "C" void app_main(void)
{
	TaskHandle_t controller_task_h = nullptr;
	xTaskCreate(
		task::controller_task,
		"controller_task",
		2048,
		nullptr,
		3,
		&controller_task_h
	);

	while (true) {
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}
