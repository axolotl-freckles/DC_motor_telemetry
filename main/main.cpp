#include <stdio.h>

#include "esp_log.h"

#include "freertos/FreeRTOS.h"

#include "controller.hpp"
#include "controllers/pid_controller.hpp"
#include "filters.hpp"

extern "C" void app_main(void)
{

	while (true) {
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}
