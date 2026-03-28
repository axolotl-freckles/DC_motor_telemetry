/**
 * @file encoder.cpp
 * @author ACMAX (aavaloscorrales@gmail.com)
 * @brief
 * @version 0.1
 * @date 2026-03-25
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "encoder_task.hpp"

#include <cstdlib>

#include "esp_log.h"
#include "esp_timer.h"

using namespace task;
using namespace task::encoder;

const char LOG_TAG[] = "encoder";

constexpr TickType_t ENCODER_TICK_TIME_ms = 2000;

constexpr int64_t IDLE_TIME_us       = 3000000;
constexpr int64_t SAMPLING_PERIOD_us = 6000000;

constexpr EventBits_t INIT_OK    = 0b1 << 11;
constexpr EventBits_t STATE_MASK = ~(INIT_OK | EncoderState_e::ERROR);

struct StateStruct_t {
	EventBits_t                   * current_state;
	StateSwitcher<EncoderState_e> * transition_handler;
};
struct TaskArgs_t {
	/* Task management variables */
	TaskHandle_t                   * _frtos_task_h;
	StaticEventGroup_t             * _encoder_state_event_group;
	EventGroupHandle_t             * _encoder_state_event_group_h;
	StateSwitcher<EncoderState_e> ** _transition_handler;
	/* Runtime variables */
	/* Message interface variables */
	QueueHandle_t                  * _speed_qh;
};

static void encoder_task_fn(void* args);

static void handle_state(const StateStruct_t &state);
static void handle_error(const StateStruct_t &state);

static void idle_loop  (const StateStruct_t &state);
static void sample_loop(const StateStruct_t &state);

void encoder_task_fn(void *args) {
	EventGroupHandle_t  encoder_state_event_group_h;
	TaskArgs_t         *interface_attr     = (TaskArgs_t*)args;
	TickType_t          previous_wake_time = xTaskGetTickCount();
	EventBits_t         current_state      = EncoderState_e::IDLE;
	StateSwitcher<EncoderState_e> *transition_handler = nullptr;

	encoder_state_event_group_h = *interface_attr->_encoder_state_event_group_h;

	transition_handler = new StateSwitcher<EncoderState_e>(
		encoder_state_event_group_h,
		STATE_MASK
	);
	transition_handler->update_state(EncoderState_e::IDLE);

	StateStruct_t state = {
		.current_state      = &current_state,
		.transition_handler =  transition_handler
	};

	*(interface_attr->_transition_handler) = transition_handler;

	xEventGroupSetBits(encoder_state_event_group_h, INIT_OK);

	while (true) {
		current_state = xEventGroupGetBits(encoder_state_event_group_h);

		if (current_state & EncoderState_e::ERROR) {
			handle_error(state);
		}
		else {
			handle_state(state);
		}

		(void)xTaskDelayUntil(
			&previous_wake_time,
			pdMS_TO_TICKS(ENCODER_TICK_TIME_ms)
		);
	}
}

void handle_state(const StateStruct_t &state) {
	switch (*state.current_state & STATE_MASK) {
		case EncoderState_e::IDLE:
			idle_loop  (state);
			break;
		case EncoderState_e::SAMPLING:
			sample_loop(state);
			break;
	}
}
void handle_error(const StateStruct_t &state) {
	EventBits_t error_state =   *state.current_state
	                          & ~(EncoderState_e::ERROR);

	// Handle error
}

void idle_loop  (const StateStruct_t &state) {
	int64_t curr_time = esp_timer_get_time();
	if (   curr_time < SAMPLING_PERIOD_us
		&& curr_time > IDLE_TIME_us
	) {
		state.transition_handler->update_state(EncoderState_e::SAMPLING);
	}
	return;
}
void sample_loop(const StateStruct_t &state) {
	float speed_sample = 200.0f + std::rand()%100;
	static int64_t start_time = 0;
	int64_t curr_time  = esp_timer_get_time();

	speed_sample /= 3.0f;
	ESP_LOGI(LOG_TAG, "speed: %.3e", speed_sample);

	if (!start_time) {
		start_time = curr_time;
	}
	else if ( curr_time > start_time+SAMPLING_PERIOD_us ) {
		state.transition_handler->update_state(EncoderState_e::IDLE);
	}
}

task::encoder::EncoderTask::EncoderTask()
:
	/* Task management variables */
	_frtos_task_h               (),
	_encoder_state_event_group  (),
	_encoder_state_event_group_h(),
	_transition_handler         (nullptr),
	/* Runtime variables */
	/* Message interface variables */
	_speed_qh                   ()
{
	TaskArgs_t args {
		/* Task management variables */
		._encoder_state_event_group_h = &_encoder_state_event_group_h,
		._transition_handler          = &_transition_handler,
		/* Runtime variables */
		/* Message interface variables */
		._speed_qh    = &_speed_qh
	};
	_encoder_state_event_group_h = xEventGroupCreateStatic(
		&_encoder_state_event_group
	);
	xTaskCreate(
		encoder_task_fn,
		"controller_task",
		2048,
		&args,
		3,
		&_frtos_task_h
	);
	xEventGroupWaitBits(
		_encoder_state_event_group_h,
		INIT_OK | EncoderState_e::ERROR,
		pdFALSE,
		pdFALSE,
		portMAX_DELAY
	);
	ESP_LOGI(LOG_TAG, "Task started!");
}

EncoderTask& task::encoder::EncoderTask::get_instance() {
	static EncoderTask singleton_encoder_task;
	return singleton_encoder_task;
}

void task::encoder::EncoderTask::set_params(const config_params& params) {
	_speed_qh = params.speed_qh;
}
