#include <stdio.h>

#include "esp_log.h"

#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sampler_task.hpp"

using task::sampler::SamplerTask;

static const char *TAG = "main";


extern "C" void app_main(void)
{
	SamplerTask *sampler_task = &SamplerTask::get_instance();
	(void)sampler_task;

	ESP_LOGI(TAG, "Encoder test started");

	while (true) {
		const float    speed_rad_s = sampler_task->current_w();
		const float    speed_rpm    = speed_rad_s * 60.0f / (2.0f * 3.14159265358979323846f);
		const uint64_t last_pulse   = sampler_task->get_encoder().getLastPulseTime();
		const uint64_t now_us       = esp_timer_get_time();
		const uint64_t age_us       = now_us - last_pulse;

		ESP_LOGI(
			TAG,
			"Encoder: rpm=%.3f rad_s=%.3f last_pulse_age=%llu us",
			speed_rpm,
			speed_rad_s,
			(unsigned long long)age_us
		);
		vTaskDelay(pdMS_TO_TICKS(100));
	}

	// The rest of the control stack is intentionally disabled for this test.
}

