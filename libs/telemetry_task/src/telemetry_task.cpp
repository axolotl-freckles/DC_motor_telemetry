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
#include <cmath>

#include "esp_timer.h"

using namespace task::telemetry;

constexpr uint64_t   SEND_WATCHDOG_TIMEOUT_us = 10000L;
constexpr TickType_t TELEMETRY_TICK_TIME_ms   = 50;

constexpr EventBits_t INIT_OK = 0b1 << 11;

struct TaskArgs_t {
	EventGroupHandle_t *state_event_group_h;
	QueueHandle_t      *data_queue_h;
};

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