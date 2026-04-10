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

constexpr EventBits_t INIT_OK           = 0b1 << 11;
constexpr EventBits_t STATE_MASK        = ~(INIT_OK | EncoderState_e::ERROR);
constexpr EventBits_t PUBLIC_STATE_MASK = ~(INIT_OK);

struct StateStruct_t {
	EventBits_t                   * current_state            = nullptr;
	StateSwitcher<EncoderState_e> * transition_handler       = nullptr;
	EventGroupHandle_t              task_state_event_group_h = nullptr;
};
struct TaskArgs_t {
	/* Task management variables */
	EventGroupHandle_t             * _task_state_event_group_h = nullptr;
	StateSwitcher<EncoderState_e> ** _transition_handler       = nullptr;
	/* Runtime variables */
	/* Message interface variables */
};
struct QueueHandles_t {
	QueueHandle_t speed_qh = nullptr;
};

static QueueHandles_t queues = {
	.speed_qh = nullptr
};

static inline const char * state_to_str(EventBits_t state_bits) {
	if (state_bits & EncoderState_e::ERROR) {
		return "ERROR";
	}
	switch (state_bits & STATE_MASK)
	{
	case EncoderState_e::IDLE:
		return "IDLE";
	case EncoderState_e::SAMPLING:
		return "SAMPLING";
	default:
		return "UNKNOWN";
	}
}

static void encoder_task_fn(void* args);

static void handle_state(const StateStruct_t &state);
static void handle_error(const StateStruct_t &state);

static void idle_loop  (const StateStruct_t &state);
static void sample_loop(const StateStruct_t &state);

static bool handle_transition(EncoderState_e from, EncoderState_e to);

void encoder_task_fn(void *args) {
	char trans_hand_mem_space[sizeof(StateSwitcher<EncoderState_e>)] = {0};
	EventGroupHandle_t  encoder_state_event_group_h;
	TaskArgs_t         *interface_attr     = (TaskArgs_t*)args;
	TickType_t          previous_wake_time = xTaskGetTickCount();
	EventBits_t         current_state      = EncoderState_e::IDLE;
	StateSwitcher<EncoderState_e> *transition_handler = nullptr;

	encoder_state_event_group_h = *interface_attr->_task_state_event_group_h;

	transition_handler = new (trans_hand_mem_space) StateSwitcher<EncoderState_e>(
		encoder_state_event_group_h,
		STATE_MASK
	);
	transition_handler->update_state(EncoderState_e::IDLE);

	StateStruct_t state = {
		.current_state            = &current_state,
		.transition_handler       =  transition_handler,
		.task_state_event_group_h =  encoder_state_event_group_h
	};

	*(interface_attr->_transition_handler) = transition_handler;

	transition_handler->set_trans_callback(handle_transition);

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
	ESP_LOGI(LOG_TAG, "Idling");
	(void)xEventGroupWaitBits(
		state.task_state_event_group_h,
		EncoderState_e::SAMPLING | EncoderState_e::ERROR,
		pdFALSE,
		pdFALSE,
		portMAX_DELAY
	);
	return;
}
void sample_loop(const StateStruct_t &state) {
	float speed_sample = 200.0f + std::rand()%100;

	speed_sample /= 3.0f;
	ESP_LOGI(LOG_TAG, "speed: %.3e", speed_sample);

	xQueueOverwrite(queues.speed_qh, &speed_sample);
}

/* ###################################################### TRANSITION HANDLING */

bool handle_transition(EncoderState_e from, EncoderState_e to) {
	ESP_LOGI(
		LOG_TAG,
		"Transitioning from %s to %s",
		state_to_str(from),
		state_to_str(to)
	);
	return true;
}

/* ########################################################## PUBLIC TASK API */

esp_err_t task::encoder::EncoderTask::start() {
	esp_err_t can_start     = ESP_OK;
	bool      transition_ok = false;
	if (!queues.speed_qh) {
		ESP_LOGE(LOG_TAG, "No speed out queue!");
		can_start = ESP_ERR_INVALID_STATE;
	}
	if (!_transition_handler) {
		can_start = ESP_ERR_INVALID_STATE;
	}
	if (ESP_OK == can_start && _transition_handler) {
		transition_ok =
			_transition_handler->update_state(EncoderState_e::SAMPLING);
		if ( !transition_ok ) {
			ESP_LOGE(LOG_TAG, "Start transition failed");
			return ESP_ERR_NOT_ALLOWED;
		}
	}
	return can_start;
}
esp_err_t task::encoder::EncoderTask::stop() {
	bool transition_ok =
		_transition_handler->update_state(EncoderState_e::IDLE);
	return transition_ok? ESP_OK : ESP_ERR_INVALID_STATE;
}

task::encoder::EncoderTask::EncoderTask()
:	StateTask()
	/* Task management variables */
	, _encoder_state_event_group  ()
	, _transition_handler         (nullptr)
	/* Runtime variables */
	/* Message interface variables */
	, _speed_qh                   ()
{
	TaskArgs_t args {
		/* Task management variables */
		._task_state_event_group_h = &_task_state_event_group_h,
		._transition_handler       = &_transition_handler,
		/* Runtime variables */
		/* Message interface variables */
	};
	_task_state_event_group_h = xEventGroupCreateStatic(
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
		_task_state_event_group_h,
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

	queues.speed_qh = _speed_qh;
}

EventBits_t task::encoder::EncoderTask::get_state() {
	return xEventGroupGetBits(_task_state_event_group_h) & PUBLIC_STATE_MASK;
}

task::encoder::EncoderTask::~EncoderTask() {}
