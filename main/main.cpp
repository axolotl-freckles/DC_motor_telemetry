#include "esp_log.h"

#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "apply_task.hpp"
#include "sampler_task.hpp"

static const char *TAG = "main";


extern "C" void app_main(void)
{
	task::sampler::SamplerTask &sampler_task = task::sampler::SamplerTask::get_instance();
	task::apply::ApplyTask &apply_task = task::apply::ApplyTask::get_instance();
	QueueHandle_t voltage_qh = task::apply::ApplyTask::createQueue(1);
	task::apply::ApplyTask::config_params apply_config = {
		.voltage_queue_h = voltage_qh
	};
	apply_task.set_params(apply_config);
	ESP_ERROR_CHECK(apply_task.start());

	ESP_LOGI(TAG, "Motor motion test started using ApplyTask + encoder");

	while (true) {
		for (uint32_t duty_pct = 10U; duty_pct <= 50U; duty_pct += 5U) {
			const float duty = duty_pct / 100.0f;
			const float voltage = 24.0f * duty / (1.0f - duty);

			xQueueOverwrite(voltage_qh, &voltage);
			vTaskDelay(pdMS_TO_TICKS(250));

			const float speed_rad_s = sampler_task.current_w();
			const float speed_rpm = speed_rad_s * 60.0f / (2.0f * 3.14159265358979323846f);
			const uint64_t last_pulse_age_us = esp_timer_get_time() - sampler_task.get_encoder().getLastPulseTime();

			ESP_LOGI(
				TAG,
				"Ramp: duty=%u%% voltage=%.3f V speed=%.3f rpm (%.3f rad/s) last_pulse=%llu us",
				(unsigned)duty_pct,
				voltage,
				speed_rpm,
				speed_rad_s,
				(unsigned long long)last_pulse_age_us
			);
			if (50U == duty_pct) {
				vTaskDelay(pdMS_TO_TICKS(1000));
			}
			else if (10U == duty_pct) {
				vTaskDelay(pdMS_TO_TICKS(1000));
			}
			else {
				vTaskDelay(pdMS_TO_TICKS(300));
			}
		}

		for (int32_t duty_pct = 45; duty_pct >= 10; duty_pct -= 5) {
			const float duty = duty_pct / 100.0f;
			const float voltage = 24.0f * duty / (1.0f - duty);

			xQueueOverwrite(voltage_qh, &voltage);
			vTaskDelay(pdMS_TO_TICKS(250));

			const float speed_rad_s = sampler_task.current_w();
			const float speed_rpm = speed_rad_s * 60.0f / (2.0f * 3.14159265358979323846f);
			const uint64_t last_pulse_age_us = esp_timer_get_time() - sampler_task.get_encoder().getLastPulseTime();

			ESP_LOGI(
				TAG,
				"Ramp down: duty=%d%% voltage=%.3f V speed=%.3f rpm (%.3f rad/s) last_pulse=%llu us",
				duty_pct,
				voltage,
				speed_rpm,
				speed_rad_s,
				(unsigned long long)last_pulse_age_us
			);
			if (10 == duty_pct) {
				vTaskDelay(pdMS_TO_TICKS(1000));
			}
			else {
				vTaskDelay(pdMS_TO_TICKS(300));
			}
		}
		ESP_LOGI(TAG, "Restarting slower PWM ramp from 10%% to 50%% and back down");
	}

	// The rest of the control stack is intentionally disabled for this test.
}

