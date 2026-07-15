/**
 * @file telemetry_task.cpp
 * @author ACMAX (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2026-06-28
 *
 * @copyright Copyright (c) 2026
 *
 */
#include "telemetry_task.hpp"

#include <cctype>
#include <cstdlib>
#include <stdio.h>
#include <cmath>
#include <string.h>

#include "esp_timer.h"
#include "esp_websocket_client.h"
#include "esp_log.h"

using namespace task::telemetry;

constexpr uint64_t   SEND_WATCHDOG_TIMEOUT_us = 10000L;
constexpr TickType_t TELEMETRY_TICK_TIME_ms   = 50;
static constexpr const char *TAG = "telemetry_task";
static constexpr const char *WS_URI = "ws://192.168.137.1:8080";
static constexpr float DUTY_SETPOINT_DEFAULT_PERCENT = 20.0f;

constexpr EventBits_t INIT_OK = 0b1 << 11;

struct TaskArgs_t {
	EventGroupHandle_t *state_event_group_h;
	QueueHandle_t      *data_queue_h;
};

static esp_websocket_client_handle_t s_ws_client = nullptr;
static QueueHandle_t s_ws_setpoint_queue_h = nullptr;

static void websocket_event_handler(
	void *handler_args,
	esp_event_base_t base,
	int32_t event_id,
	void *event_data
) {
	(void)handler_args;
	(void)base;
	(void)event_data;

	if (event_id == WEBSOCKET_EVENT_CONNECTED) {
		ESP_LOGI(TAG, "WebSocket connected");
	}
	else if (event_id == WEBSOCKET_EVENT_DISCONNECTED) {
		ESP_LOGW(TAG, "WebSocket disconnected");
	}
	else if (event_id == WEBSOCKET_EVENT_DATA) {
		esp_websocket_event_data_t *data = (esp_websocket_event_data_t *)event_data;
		if ((s_ws_setpoint_queue_h != nullptr) && (data != nullptr) && (data->data_ptr != nullptr) && (data->data_len > 0)) {
			char rx_buffer[32] = {0};
			int copy_len = data->data_len;
			if (copy_len >= static_cast<int>(sizeof(rx_buffer))) {
				copy_len = static_cast<int>(sizeof(rx_buffer) - 1);
			}

			memcpy(rx_buffer, data->data_ptr, copy_len);
			rx_buffer[copy_len] = '\0';

			for (int i = 0; i < copy_len; ++i) {
				if (rx_buffer[i] == ',') {
					rx_buffer[i] = '.';
				}
			}

			char *start = rx_buffer;
			while ((*start != '\0') && std::isspace(static_cast<unsigned char>(*start))) {
				start++;
			}

			char *endptr = nullptr;
			const float duty_percent = strtof(start, &endptr);

			while ((endptr != nullptr) && (*endptr != '\0') && std::isspace(static_cast<unsigned char>(*endptr))) {
				endptr++;
			}

			if ((start != endptr) && (endptr != nullptr) && (*endptr == '\0')) {
				(void)xQueueOverwrite(s_ws_setpoint_queue_h, &duty_percent);
				ESP_LOGI(TAG, "WS duty setpoint received: %.2f %%", duty_percent);
			}
		}
	}
}

static void websocket_app_start(void) {
	esp_websocket_client_config_t websocket_cfg = {};
	websocket_cfg.uri = WS_URI;

	s_ws_client = esp_websocket_client_init(&websocket_cfg);
	if (s_ws_client == nullptr) {
		ESP_LOGE(TAG, "esp_websocket_client_init failed");
		return;
	}

	esp_websocket_register_events(
		s_ws_client,
		WEBSOCKET_EVENT_ANY,
		websocket_event_handler,
		(void *)s_ws_client
	);

	esp_err_t err = esp_websocket_client_start(s_ws_client);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "esp_websocket_client_start failed: %s", esp_err_to_name(err));
	}
}

static void telemetry_task_fn(void *args);

static inline void clear_data (telemetry_data_t &data) {
	data.timestamp      = 0.0f;
	data.setpoint       = 0.0f;
	data.set_voltage    = 0.0f;
	data.w_rad_s        = 0.0f;
	data.I_amp          = 0.0f;
	data.estimated_load = 0.0f;
}

void telemetry_task_fn(void *args) {
	TaskArgs_t received_args = *(TaskArgs_t*)args;
	EventGroupHandle_t state_event_group_h = *received_args.state_event_group_h;
	QueueHandle_t      data_queue_h        = *received_args.data_queue_h;
	TickType_t         previous_wake_time  = xTaskGetTickCount();

	telemetry_data_t received_data;

	xEventGroupSetBits(state_event_group_h, INIT_OK);

	while (true)
	{
		//uint64_t trans_start = esp_timer_get_time();
		clear_data (received_data);
		(void)xQueueReceive(data_queue_h, &received_data, portMAX_DELAY);
		(void)printf(
			 "%10.3e"
			",%10.3e"
			",%10.3e"
			",%10.3e"
			",%10.3e"
			",%10.3e\n",
			received_data.timestamp,
			received_data.setpoint,
			received_data.set_voltage,
			received_data.w_rad_s * 60.0f/(2.0f*M_PI),
			received_data.I_amp,
			received_data.estimated_load
		);

		if ((s_ws_client != nullptr) && esp_websocket_client_is_connected(s_ws_client)) {
			char tx[192];
			int len = snprintf(
				tx,
				sizeof(tx),
				"%10.3e,%10.3e,%10.3e,%10.3e,%10.3e,%10.3e\n",
				received_data.timestamp,
				received_data.setpoint,
				received_data.set_voltage,
				received_data.w_rad_s * 60.0f/(2.0f*M_PI),
				received_data.I_amp,
				received_data.estimated_load
			);
			if (len > 0) {
				esp_websocket_client_send_text(
					s_ws_client,
					tx,
					len,
					0
				);
			}
		}

		//(void)xTaskDelayUntil(
		//	&previous_wake_time,
		//	pdMS_TO_TICKS(TELEMETRY_TICK_TIME_ms)
		//);
	}
}

/* ###################################################### TRANSITION HANDLING */

/* ########################################################## PUBLIC TASK API */

task::telemetry::TelemetryTask::TelemetryTask() {
	_task_state_event_group_h = xEventGroupCreateStatic (
		&_telemetry_state_event_group
	);
	_data_queue_h = xQueueCreate(DATA_QUEUE_LEN, sizeof(telemetry_data_t));
	_ws_setpoint_queue_h = xQueueCreate(1, sizeof(float));
	s_ws_setpoint_queue_h = _ws_setpoint_queue_h;
	if (_ws_setpoint_queue_h != nullptr) {
		float default_percent = DUTY_SETPOINT_DEFAULT_PERCENT;
		(void)xQueueOverwrite(_ws_setpoint_queue_h, &default_percent);
	}
	websocket_app_start();

	TaskArgs_t args {
		.state_event_group_h = &_task_state_event_group_h,
		.data_queue_h        = &_data_queue_h
	};
	xTaskCreate(
		telemetry_task_fn,
		"telemetry_task",
		2048 + 512,
		&args,
		3,
		&_frtos_task_h
	);
	xEventGroupWaitBits(
		_task_state_event_group_h,
		INIT_OK,
		pdFALSE,
		pdFALSE,
		portMAX_DELAY
	);
}

TelemetryTask& task::telemetry::TelemetryTask::get_instance() {
	static TelemetryTask telemetry_task;
	return telemetry_task;
}