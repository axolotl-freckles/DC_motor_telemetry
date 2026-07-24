/**
 * @file monitor_input.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2026-07-12
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#include "monitor_input.hpp"

#include <cctype>
#include <cmath>
#include <cstdlib>
#include <algorithm>

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "sampler_task.hpp"
#include "controller_task.hpp"

using namespace task::monitor_in;

using task::controller::ControllerTask;
using task::sampler::SamplerTask;

static const char *TAG = "monitor_in";

static constexpr float TWO_PI                = 2.0f*M_PI;
static constexpr float VOLTAGE_BATTERY       = 25.0f;
static constexpr float DUTY_MIN              = 0.10f;
static constexpr float DUTY_MAX              = 0.90f;
static constexpr int   LOG_PERIOD_ms         = 1000;
static constexpr int   WAIT_LOG_PERIOD_ms    = 3000;
static constexpr int   INPUT_POLL_ms         = 100;
static constexpr int   INPUT_LINE_TIMEOUT_ms = 1200;
static constexpr float SETPOINT_MIN_RPM      = 0.0f;
static constexpr float SETPOINT_MAX_RPM      = 200.0f;
static constexpr float REACHED_TOLERANCE_RPM = 3.0f;

static StaticEventGroup_t monitor_state_event_group;
static EventGroupHandle_t state_event_gh = nullptr;
static QueueHandle_t      setpoint_qh    = nullptr;

static void monitor_input_task_fn(void *args) {
	xEventGroupSetBits  (state_event_gh, MonitorInputState_e::IDLE);
	xEventGroupWaitBits (
		state_event_gh,
		MonitorInputState_e::RECEIVING,
		pdFALSE,
		pdFALSE,
		portMAX_DELAY
	);
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
		if (MonitorInput::try_read_setpoint_rpm(requested_setpoint_rpm)) {
			const float clamped_setpoint = std::clamp(
				requested_setpoint_rpm,
				SETPOINT_MIN_RPM,
				SETPOINT_MAX_RPM
			);
			active_setpoint_rpm = clamped_setpoint;
			has_active_setpoint = true;

			//const float active_setpoint_rad_s = active_setpoint_rpm * TWO_PI / 60.0f;
			const float active_setpoint_rad_s = active_setpoint_rpm;
			xQueueOverwrite(setpoint_qh, &active_setpoint_rad_s);

			if (!controller_started) {
				ESP_ERROR_CHECK(ControllerTask::get_instance().start());
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

				const float speed_rad_s = SamplerTask::get_instance().current_w();
				const float speed_rpm = speed_rad_s * 60.0f / TWO_PI;
				const uint64_t last_pulse_age_us = esp_timer_get_time() - SamplerTask::get_instance().get_encoder().getLastPulseTime();

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

			const float speed_rad_s = SamplerTask::get_instance().current_w();
			const float speed_rpm = speed_rad_s * 60.0f / TWO_PI;
			const float target_rpm = has_active_setpoint ? active_setpoint_rpm : 0.0f;
			const float speed_error_rpm = target_rpm - speed_rpm;
			const bool reached = std::fabs(speed_error_rpm) <= REACHED_TOLERANCE_RPM;
			const uint64_t last_pulse_age_us = esp_timer_get_time() - SamplerTask::get_instance().get_encoder().getLastPulseTime();

			ESP_LOGD(
				TAG,
				"%s tgt=%.1f rpm | act=%.1f rpm | err=%.1f rpm | reached=%s | pulse_age=%.1f ms",
				has_active_setpoint ? "MANUAL" : "WAIT_SP",
				target_rpm,
				speed_rpm,
				speed_error_rpm,
				reached ? "YES" : "NO",
				last_pulse_age_us * 1e-3f
			);
		}

		vTaskDelay(pdMS_TO_TICKS(INPUT_POLL_ms));
	}
}

bool task::monitor_in::MonitorInput::try_read_setpoint_rpm(float &setpoint_rpm_out)
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

/* ########################################################## PUBLIC TASK API */

esp_err_t task::monitor_in::MonitorInput::start() {
	esp_err_t   can_start  = ESP_OK;
	EventBits_t curr_state = xEventGroupGetBits(_task_state_event_group_h);
	if ( !setpoint_qh ) {
		ESP_LOGE(TAG, "No setpoint queue handler!");
		can_start = ESP_ERR_INVALID_STATE;
	}
	if (ESP_OK == can_start) {
		xEventGroupClearBits(_task_state_event_group_h, curr_state);
		xEventGroupSetBits  (_task_state_event_group_h, MonitorInputState_e::RECEIVING);
	}
	return can_start;
}
esp_err_t MonitorInput::stop()  { return ESP_OK; }

MonitorInput& task::monitor_in::MonitorInput::get_instance() {
	static MonitorInput task_instance;
	return task_instance;
}

void task::monitor_in::MonitorInput::set_setpoint_qh(QueueHandle_t _setpoint_qh) {
	setpoint_qh = _setpoint_qh;
}

task::monitor_in::MonitorInput::MonitorInput() {
	/*      FreeRTOS Task      */
	_task_state_event_group_h = xEventGroupCreateStatic (
		&monitor_state_event_group
	);
	state_event_gh = _task_state_event_group_h;
	xEventGroupClearBits(
		_task_state_event_group_h,
		(
			  MonitorInputState_e::IDLE
			| MonitorInputState_e::RECEIVING
			| MonitorInputState_e::ERROR
		)
	);

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

	xTaskCreate (
		monitor_input_task_fn,
		"monitor_in_task",
		2048 + 1024,
		nullptr,
		3,
		&_frtos_task_h
	);
	xEventGroupWaitBits (
		_task_state_event_group_h,
		MonitorInputState_e::ERROR | MonitorInputState_e::IDLE,
		pdFALSE,
		pdFALSE,
		portMAX_DELAY
	);
	ESP_LOGI(TAG, "Task started!");
}
