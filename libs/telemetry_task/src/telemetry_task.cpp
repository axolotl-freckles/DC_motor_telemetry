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

#include <stdio.h>
#include <string.h>

#include "esp_timer.h"
#include "esp_websocket_client.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_event.h"

using namespace task::telemetry;

constexpr uint64_t   SEND_WATCHDOG_TIMEOUT_us = 10000L;
constexpr TickType_t TELEMETRY_TICK_TIME_ms   = 50;

static constexpr const char *TAG = "telemetry_task";
static constexpr const char *WS_URI = "ws://192.168.137.1:8080";

constexpr EventBits_t INIT_OK = 0b1 << 11;

struct TaskArgs_t {
	EventGroupHandle_t *state_event_group_h;
	QueueHandle_t      *data_queue_h;
};

static TaskArgs_t s_task_args = {};
static esp_websocket_client_handle_t s_ws_client = nullptr;

static void telemetry_task_fn(void *args);
static void websocket_app_start(void);
static void websocket_event_handler(
	void *handler_args,
	esp_event_base_t base,
	int32_t event_id,
	void *event_data
);

static void log_error_if_nonzero(const char *message, int error_code) {
	if (error_code != 0) {
		ESP_LOGE(TAG, "%s: 0x%x", message, error_code);
	}
}

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
			// ",%10.3e"
			// ",%10.3e"
			",%10.3e\n",
			received_data.timestamp,
			received_data.setpoint,
			received_data.set_voltage,
			received_data.w_rad_s//,
			// received_data.I_amp,
			// received_data.estimated_load
		);

		if ((s_ws_client != nullptr) && esp_websocket_client_is_connected(s_ws_client)) {
            char tx[128];

            snprintf(tx, sizeof(tx), 
				"%10.6e,%10.6e",
                received_data.timestamp,
                received_data.w_rad_s
            );

            esp_websocket_client_send_text(
				s_ws_client,
                tx,
                strlen(tx),
                portMAX_DELAY
            );
        }

		//(void)xTaskDelayUntil(
		//	&previous_wake_time,
		//	pdMS_TO_TICKS(TELEMETRY_TICK_TIME_ms)
		//);
	}
}

static void websocket_event_handler(
	void *handler_args,
	esp_event_base_t base,
	int32_t event_id,
	void *event_data
) {
	(void)handler_args;
	(void)base;
	esp_websocket_event_data_t *data = (esp_websocket_event_data_t *)event_data;

	switch (event_id) {
		case WEBSOCKET_EVENT_CONNECTED:
			ESP_LOGI(TAG, "WEBSOCKET_EVENT_CONNECTED");
			break;
		case WEBSOCKET_EVENT_DISCONNECTED:
			ESP_LOGW(TAG, "WEBSOCKET_EVENT_DISCONNECTED");
			log_error_if_nonzero("HTTP status code", data->error_handle.esp_ws_handshake_status_code);
			if (data->error_handle.error_type == WEBSOCKET_ERROR_TYPE_TCP_TRANSPORT) {
				log_error_if_nonzero("reported from esp-tls", data->error_handle.esp_tls_last_esp_err);
				log_error_if_nonzero("reported from tls stack", data->error_handle.esp_tls_stack_err);
				log_error_if_nonzero("socket errno", data->error_handle.esp_transport_sock_errno);
			}
			break;
		case WEBSOCKET_EVENT_DATA:
			ESP_LOGD(TAG, "WEBSOCKET_EVENT_DATA len=%d op=%d", data->data_len, data->op_code);
			break;
		case WEBSOCKET_EVENT_ERROR:
			ESP_LOGE(TAG, "WEBSOCKET_EVENT_ERROR");
			log_error_if_nonzero("HTTP status code", data->error_handle.esp_ws_handshake_status_code);
			if (data->error_handle.error_type == WEBSOCKET_ERROR_TYPE_TCP_TRANSPORT) {
				log_error_if_nonzero("reported from esp-tls", data->error_handle.esp_tls_last_esp_err);
				log_error_if_nonzero("reported from tls stack", data->error_handle.esp_tls_stack_err);
				log_error_if_nonzero("socket errno", data->error_handle.esp_transport_sock_errno);
			}
			break;
		default:
			ESP_LOGD(TAG, "WEBSOCKET_EVENT id=%ld", (long)event_id);
			break;
	}
}

static void websocket_app_start(void) {
	esp_err_t err = esp_netif_init();
	if ((err != ESP_OK) && (err != ESP_ERR_INVALID_STATE)) {
		ESP_LOGE(TAG, "esp_netif_init failed: %s", esp_err_to_name(err));
		return;
	}

	err = esp_event_loop_create_default();
	if ((err != ESP_OK) && (err != ESP_ERR_INVALID_STATE)) {
		ESP_LOGE(TAG, "esp_event_loop_create_default failed: %s", esp_err_to_name(err));
		return;
	}

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

	err = esp_websocket_client_start(s_ws_client);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "esp_websocket_client_start failed: %s", esp_err_to_name(err));
		return;
	}

	ESP_LOGI(TAG, "WebSocket client started. URI=%s", WS_URI);
}

/* ###################################################### TRANSITION HANDLING */

/* ########################################################## PUBLIC TASK API */

task::telemetry::TelemetryTask::TelemetryTask() {
	_task_state_event_group_h = xEventGroupCreateStatic (
		&_telemetry_state_event_group
	);
	_data_queue_h = xQueueCreate(DATA_QUEUE_LEN, sizeof(telemetry_data_t));
	websocket_app_start();

	s_task_args = TaskArgs_t {
		.state_event_group_h = &_task_state_event_group_h,
		.data_queue_h        = &_data_queue_h
	};
	xTaskCreate(
		telemetry_task_fn,
		"telemetry_task",
		4096,
		&s_task_args,
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