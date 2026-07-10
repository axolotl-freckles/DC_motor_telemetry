#include "esp_log.h"

#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <cmath>

#include "controller_task.hpp"
#include "apply_task.hpp"
#include "sampler_task.hpp"

static const char *TAG = "main";
static constexpr float TWO_PI = 6.2831853071795864769f;
static constexpr float VOLTAGE_BATTERY = 25.0f;
static constexpr float DUTY_MIN = 0.10f;
static constexpr float DUTY_MAX = 0.90f;
static constexpr int HOLD_DURATION_ms = 50000;
static constexpr int LOG_PERIOD_ms = 1000;
static constexpr int SETPOINT_RAMP_ms = 4000;
static constexpr float REACHED_TOLERANCE_RPM = 3.0f;

static float clampf(float x, float lo, float hi)
{
	if (x < lo) {
		return lo;
	}
	if (x > hi) {
		return hi;
	}
	return x;
}

extern "C" void app_main(void)
{
	task::controller::ControllerTask &controller_task =
		task::controller::ControllerTask::get_instance();
	task::sampler::SamplerTask &sampler_task =
		task::sampler::SamplerTask::get_instance();
	task::apply::ApplyTask &apply_task =
		task::apply::ApplyTask::get_instance();

	QueueHandle_t setpoint_qh = xQueueCreate(1, sizeof(float));
	QueueHandle_t speed_qh    = xQueueCreate(1, sizeof(float));
	QueueHandle_t cpoint_qh   = xQueueCreate(1, sizeof(float));

	task::controller::ControllerTask::config_params controller_config = {
		.setpoint_qh       = setpoint_qh,
		.speed_qh          = speed_qh,
		.control_signal_qh = cpoint_qh
	};
	task::sampler::SamplerTask::config_params sampler_config = {
		.speed_qh = speed_qh
	};
	task::apply::ApplyTask::config_params apply_config = {
		.voltage_queue_h = cpoint_qh
	};

	controller_task.set_params(controller_config);
	sampler_task.set_params(sampler_config);
	apply_task.set_params(apply_config);

	constexpr float setpoint_low_rpm = 40.0f;
	constexpr float setpoint_high_rpm = 120.0f;
	const float setpoint_low_rad_s = setpoint_low_rpm * TWO_PI / 60.0f;
	const float setpoint_high_rad_s = setpoint_high_rpm * TWO_PI / 60.0f;
	const int hold_samples = std::max(1, HOLD_DURATION_ms / LOG_PERIOD_ms);
	const int ramp_samples = std::max(1, (SETPOINT_RAMP_ms + LOG_PERIOD_ms - 1) / LOG_PERIOD_ms);

	float setpoint_init_rad_s = setpoint_low_rad_s;
	xQueueOverwrite(setpoint_qh, &setpoint_init_rad_s);
	ESP_ERROR_CHECK(controller_task.start());

	ESP_LOGI(
		TAG,
		"PID RPM cyclic test: hold LOW=%.1f rpm (%d s), ramp (%d s), hold HIGH=%.1f rpm (%d s), ramp down (%d s)",
		setpoint_low_rpm,
		HOLD_DURATION_ms / 1000,
		SETPOINT_RAMP_ms / 1000,
		setpoint_high_rpm,
		HOLD_DURATION_ms / 1000,
		SETPOINT_RAMP_ms / 1000
	);

	auto publish_and_log = [&](const char *phase, float target_rpm, float target_rad_s) {
		xQueueOverwrite(setpoint_qh, &target_rad_s);

		float control_voltage = 0.0f;
		(void)xQueuePeek(cpoint_qh, &control_voltage, 0);

		const float speed_rad_s = sampler_task.current_w();
		const float speed_rpm = speed_rad_s * 60.0f / TWO_PI;
		const float speed_error_rpm = target_rpm - speed_rpm;
		const bool reached = std::fabs(speed_error_rpm) <= REACHED_TOLERANCE_RPM;
		const uint64_t last_pulse_age_us = esp_timer_get_time() - sampler_task.get_encoder().getLastPulseTime();
		const float duty_est = clampf(control_voltage / (control_voltage + VOLTAGE_BATTERY), DUTY_MIN, DUTY_MAX);

		ESP_LOGI(
			TAG,
			"%s tgt=%.1f rpm | act=%.1f rpm | err=%.1f rpm | reached=%s | u=%.2f V | duty~=%.1f%% | pulse_age=%.1f ms",
			phase,
			target_rpm,
			speed_rpm,
			speed_error_rpm,
			reached ? "YES" : "NO",
			control_voltage,
			duty_est * 100.0f,
			last_pulse_age_us * 1e-3f
		);

		vTaskDelay(pdMS_TO_TICKS(LOG_PERIOD_ms));
	};

	while (true) {
		for (int i = 0; i < hold_samples; i++) {
			publish_and_log("LOW_HOLD", setpoint_low_rpm, setpoint_low_rad_s);
		}

		for (int i = 0; i < ramp_samples; i++) {
			const float alpha = static_cast<float>(i + 1) / static_cast<float>(ramp_samples);
			const float target_rpm = setpoint_low_rpm + alpha * (setpoint_high_rpm - setpoint_low_rpm);
			const float target_rad_s = setpoint_low_rad_s + alpha * (setpoint_high_rad_s - setpoint_low_rad_s);
			publish_and_log("RAMP_UP", target_rpm, target_rad_s);
		}

		for (int i = 0; i < hold_samples; i++) {
			publish_and_log("HIGH_HOLD", setpoint_high_rpm, setpoint_high_rad_s);
		}

		for (int i = 0; i < ramp_samples; i++) {
			const float alpha = static_cast<float>(i + 1) / static_cast<float>(ramp_samples);
			const float target_rpm = setpoint_high_rpm + alpha * (setpoint_low_rpm - setpoint_high_rpm);
			const float target_rad_s = setpoint_high_rad_s + alpha * (setpoint_low_rad_s - setpoint_high_rad_s);
			publish_and_log("RAMP_DOWN", target_rpm, target_rad_s);
		}
	}

	// Keep task alive after test to preserve logs and state.
}

