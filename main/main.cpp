#include "esp_log.h"

#include "esp_timer.h"

#include "driver/uart.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <cctype>
#include <cmath>
#include <cstdlib>

#include "controller_task.hpp"
#include "apply_task.hpp"
#include "sampler_task.hpp"

using task::controller::ControllerTask;
using task::controller::ControllerState_e;
using task::sampler::SamplerTask;
using task::apply::ApplyTask;

static const char *TAG = "main";
static constexpr float TWO_PI = 6.2831853071795864769f;
static constexpr float VOLTAGE_BATTERY = 25.0f;
static constexpr float DUTY_MIN = 0.10f;
static constexpr float DUTY_MAX = 0.90f;
static constexpr int LOG_PERIOD_ms = 1000;
static constexpr int WAIT_LOG_PERIOD_ms = 3000;
static constexpr int INPUT_POLL_ms = 100;
static constexpr int INPUT_LINE_TIMEOUT_ms = 1200;
static constexpr float SETPOINT_MIN_RPM = 0.0f;
static constexpr float SETPOINT_MAX_RPM = 200.0f;
static constexpr float REACHED_TOLERANCE_RPM = 3.0f;

static bool try_read_setpoint_rpm(float &setpoint_rpm_out)
{
	static char rx_line[32] = {0};
	static int rx_len = 0;
	static int64_t last_rx_us = 0;

	uint8_t buffer[32] = {0};
	const int bytes_read = uart_read_bytes(UART_NUM_0, buffer, sizeof(buffer), 0);
	bool should_parse = false;

	if (bytes_read > 0) {
		for (int i = 0; i < bytes_read; i++) {
			char c = static_cast<char>(buffer[i]);

			if (c == ',') {
				c = '.';
			}

			if (c == '\r' || c == '\n') {
				if (rx_len > 0) {
					should_parse = true;
				}
				continue;
			}

			if (!std::isprint(static_cast<unsigned char>(c)) && c != '\t') {
				continue;
			}

			if (rx_len < static_cast<int>(sizeof(rx_line) - 1)) {
				rx_line[rx_len++] = c;
				last_rx_us = esp_timer_get_time();
			} else {
				should_parse = true;
			}
		}
	}

	if (!should_parse && rx_len > 0) {
		const int64_t now_us = esp_timer_get_time();
		if (now_us - last_rx_us >= static_cast<int64_t>(INPUT_LINE_TIMEOUT_ms) * 1000LL) {
			should_parse = true;
		}
	}

	if (!should_parse || rx_len == 0) {
		return false;
	}

	rx_line[rx_len] = '\0';

	char *start = rx_line;
	while (*start != '\0' && std::isspace(static_cast<unsigned char>(*start))) {
		start++;
	}

	char *endptr = nullptr;
	const float parsed = strtof(start, &endptr);

	while (endptr != nullptr && *endptr != '\0' && std::isspace(static_cast<unsigned char>(*endptr))) {
		endptr++;
	}

	rx_len = 0;
	rx_line[0] = '\0';

	if (start == endptr || endptr == nullptr || *endptr != '\0') {
		return false;
	}

	setpoint_rpm_out = parsed;
	return true;
}

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
	QueueHandle_t cpoint_qh   = xQueueCreate(1, sizeof(float));

	task::controller::ControllerTask::config_params controller_config = {
		.setpoint_qh       = setpoint_qh,
		.control_signal_qh = cpoint_qh
	};
	SamplerTask::config_params sampler_config = { };
	ApplyTask::config_params apply_config = {
		.voltage_queue_h = cpoint_qh
	};

	controller_task.set_params(controller_config);
	sampler_task.set_params(sampler_config);
	apply_task.set_params(apply_config);

	uart_config_t uart_config = {
		.baud_rate = 115200,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.rx_flow_ctrl_thresh = 0,
		.source_clk = UART_SCLK_DEFAULT,
		.flags = {}
	};
	ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
	esp_err_t uart_install_result = uart_driver_install(UART_NUM_0, 2048, 0, 0, nullptr, 0);
	if (uart_install_result != ESP_OK && uart_install_result != ESP_ERR_INVALID_STATE) {
		ESP_ERROR_CHECK(uart_install_result);
	}

	float setpoint_init_rad_s = 0.0f;
	xQueueOverwrite(setpoint_qh, &setpoint_init_rad_s);

	ESP_LOGI(
		TAG,
		"Manual RPM test ready. Send numeric setpoint in TeraTerm + Enter (range %.0f..%.0f rpm).",
		SETPOINT_MIN_RPM,
		SETPOINT_MAX_RPM
	);
	ESP_LOGI(TAG, "Controller is SAFE-IDLE until first setpoint. Example input: 45 or 120.");
	ESP_LOGI(TAG, "Accepted formats: 10, 10.5, 10,5, with optional spaces.");

	float active_setpoint_rpm = 0.0f;
	bool has_active_setpoint = false;
	bool controller_started = false;
	int64_t last_log_us = 0;
	int64_t last_wait_log_us = 0;

	while (true) {
		float requested_setpoint_rpm = 0.0f;
		if (try_read_setpoint_rpm(requested_setpoint_rpm)) {
			const float clamped_setpoint = clampf(
				requested_setpoint_rpm,
				SETPOINT_MIN_RPM,
				SETPOINT_MAX_RPM
			);
			active_setpoint_rpm = clamped_setpoint;
			has_active_setpoint = true;

			const float active_setpoint_rad_s = active_setpoint_rpm * TWO_PI / 60.0f;
			xQueueOverwrite(setpoint_qh, &active_setpoint_rad_s);

			if (!controller_started) {
				ESP_ERROR_CHECK(controller_task.start());
				controller_started = true;
				ESP_LOGW(TAG, "Controller armed after first setpoint.");
			}

			if (std::fabs(clamped_setpoint - requested_setpoint_rpm) > 1e-3f) {
				ESP_LOGW(
					TAG,
					"Requested setpoint %.2f rpm out of range, clamped to %.2f rpm",
					requested_setpoint_rpm,
					clamped_setpoint
				);
			} else {
				ESP_LOGI(TAG, "New setpoint accepted: %.2f rpm", active_setpoint_rpm);
			}
		}

		if (!controller_started) {
			const int64_t now_us = esp_timer_get_time();
			if (now_us - last_wait_log_us >= static_cast<int64_t>(WAIT_LOG_PERIOD_ms) * 1000LL) {
				last_wait_log_us = now_us;

				const float speed_rad_s = sampler_task.current_w();
				const float speed_rpm = speed_rad_s * 60.0f / TWO_PI;
				const uint64_t last_pulse_age_us = esp_timer_get_time() - sampler_task.get_encoder().getLastPulseTime();

				ESP_LOGW(
					TAG,
					"WAIT_SP SAFE-IDLE | type RPM + Enter | act=%.1f rpm | pulse_age=%.1f ms",
					speed_rpm,
					last_pulse_age_us * 1e-3f
				);
			}

			vTaskDelay(pdMS_TO_TICKS(INPUT_POLL_ms));
			continue;
		}

		const int64_t now_us = esp_timer_get_time();
		if (now_us - last_log_us >= static_cast<int64_t>(LOG_PERIOD_ms) * 1000LL) {
			last_log_us = now_us;

			if (has_active_setpoint) {
				const float active_setpoint_rad_s = active_setpoint_rpm * TWO_PI / 60.0f;
				xQueueOverwrite(setpoint_qh, &active_setpoint_rad_s);
			}

		float control_voltage = 0.0f;
		(void)xQueuePeek(cpoint_qh, &control_voltage, 0);

		const float speed_rad_s = sampler_task.current_w();
		const float speed_rpm = speed_rad_s * 60.0f / TWO_PI;
		const float target_rpm = has_active_setpoint ? active_setpoint_rpm : 0.0f;
		const float speed_error_rpm = target_rpm - speed_rpm;
		const bool reached = std::fabs(speed_error_rpm) <= REACHED_TOLERANCE_RPM;
		const uint64_t last_pulse_age_us = esp_timer_get_time() - sampler_task.get_encoder().getLastPulseTime();
		float duty_est = 0.0f;
		if (control_voltage > 0.0f) {
			duty_est = clampf(control_voltage / (control_voltage + VOLTAGE_BATTERY), DUTY_MIN, DUTY_MAX);
		}

		ESP_LOGI(
			TAG,
			"%s tgt=%.1f rpm | act=%.1f rpm | err=%.1f rpm | reached=%s | u=%.2f V | duty~=%.1f%% | pulse_age=%.1f ms",
			has_active_setpoint ? "MANUAL" : "WAIT_SP",
			target_rpm,
			speed_rpm,
			speed_error_rpm,
			reached ? "YES" : "NO",
			control_voltage,
			duty_est * 100.0f,
			last_pulse_age_us * 1e-3f
		);
		}

		vTaskDelay(pdMS_TO_TICKS(INPUT_POLL_ms));
	}

	// Keep task alive after test to preserve logs and state.
}

