#include <stdio.h>
#include <functional>

#include "freertos/FreeRTOS.h"

#include "controller.hpp"
#include "controllers/pid_controller.hpp"
#include "filters.hpp"

float err_func() {
	return 5.0f;
}

extern "C" void app_main(void)
{
	Filter     * const test_filter     = new LowPassRC(0.002f, 0.01);
	Controller * const test_controller = new PID(err_func, 3.0f, 2.0f, 1.0f);

	test_controller->setup();

	while (true) {
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}
