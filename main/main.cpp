#include <stdio.h>

#include "esp_log.h"

#include "freertos/FreeRTOS.h"

#include "controller.hpp"
#include "controllers/pid_controller.hpp"
#include "filters.hpp"

#include "tasks.hpp"
#include "controller_task.hpp"

using task::controller::ControllerTask;

extern "C" void app_main(void)
{
	ControllerTask controller_task = ControllerTask::get_instance();

	while (true) {
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}
