#include <stdio.h>

#include "freertos/FreeRTOS.h"

#include "filters.hpp"

extern "C" void app_main(void)
{
	Filter * const test_filter = new LowPassRC(0.002f, 0.01);

	while (true) {
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}