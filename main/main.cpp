#include <stdio.h>

#include "freertos/FreeRTOS.h"

#include "buckboost.hpp"

extern "C" void app_main(void)
{
	// BuckBoost test
	BuckBoost bb(18, 21, 12.0f);
	bb.init();
	bb.setTargetVoltage(8.0f);

	while (true) {
		vTaskDelay(3000 / portTICK_PERIOD_MS);
		bb.setTargetVoltage(12.0f);
		vTaskDelay(3000 / portTICK_PERIOD_MS);
		bb.setTargetVoltage(24.0f);
		vTaskDelay(3000 / portTICK_PERIOD_MS);
		bb.setTargetVoltage(5.0f);
	}
}
