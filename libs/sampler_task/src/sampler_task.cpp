/**
 * @file sampler_task.cpp
 * @author ACMAX (aavaloscorrales@gmail.com)
 * @brief
 * @version 0.1
 * @date 2026-03-25
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "sampler_task.hpp"

#include <cstdlib>

#include "esp_log.h"
#include "esp_timer.h"

using namespace task;
using namespace task::sampler;

const char LOG_TAG[] = "sampler";

constexpr TickType_t ENCODER_TICK_TIME_ms = 2000;

constexpr EventBits_t INIT_OK           = 0b1 << 11;
constexpr EventBits_t STATE_MASK        = ~(INIT_OK | SamplerState_e::ERROR);
constexpr EventBits_t PUBLIC_STATE_MASK = ~(INIT_OK);

struct StateStruct_t {
	EventBits_t * current_state = nullptr;
};

/* Task management variables */
static StaticEventGroup_t             sampler_state_event_group;
static EventGroupHandle_t             sampler_state_event_group_h = nullptr;

static StateSwitcher<SamplerState_e> *transition_handler = nullptr;

/* Message interface variables */
static QueueHandle_t                  speed_qh           = nullptr;

/* Runtime variables */

static inline const char * state_to_str(EventBits_t state_bits) {
	if (state_bits & SamplerState_e::ERROR) {
		return "ERROR";
	}
	switch (state_bits & STATE_MASK)
	{
	case SamplerState_e::IDLE:
		return "IDLE";
	case SamplerState_e::SAMPLING:
		return "SAMPLING";
	default:
		return "UNKNOWN";
	}
}

static void sampler_task_fn(void* args);

static void handle_state(const StateStruct_t &state);
static void handle_error(const StateStruct_t &state);

static void idle_loop  (const StateStruct_t &state);
static void sample_loop(const StateStruct_t &state);

static bool handle_transition(SamplerState_e from, SamplerState_e to);

void sampler_task_fn(void *args) {
	char trans_hand_mem_space[sizeof(StateSwitcher<SamplerState_e>)] = {0};
	TickType_t          previous_wake_time = xTaskGetTickCount();
	EventBits_t         current_state      = SamplerState_e::IDLE;

	transition_handler = new (trans_hand_mem_space) StateSwitcher<SamplerState_e>(
		sampler_state_event_group_h,
		STATE_MASK
	);
	transition_handler->update_state(SamplerState_e::IDLE);

	StateStruct_t state = {
		.current_state            = &current_state
	};

	transition_handler->set_trans_callback(handle_transition);

	xEventGroupSetBits(sampler_state_event_group_h, INIT_OK);

	while (true) {
		current_state = xEventGroupGetBits(sampler_state_event_group_h);

		if (current_state & SamplerState_e::ERROR) {
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
		case SamplerState_e::IDLE:
			idle_loop  (state);
			break;
		case SamplerState_e::SAMPLING:
			sample_loop(state);
			break;
	}
}
void handle_error(const StateStruct_t &state) {
	EventBits_t error_state =   *state.current_state
	                          & ~(SamplerState_e::ERROR);

	// Handle error
}

void idle_loop  (const StateStruct_t &state) {
	ESP_LOGI(LOG_TAG, "Idling");
	(void)xEventGroupWaitBits(
		sampler_state_event_group_h,
		SamplerState_e::SAMPLING | SamplerState_e::ERROR,
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

	xQueueOverwrite(speed_qh, &speed_sample);
}

/* ###################################################### TRANSITION HANDLING */

bool handle_transition(SamplerState_e from, SamplerState_e to) {
	ESP_LOGI(
		LOG_TAG,
		"Transitioning from %s to %s",
		state_to_str(from),
		state_to_str(to)
	);
	return true;
}

/* ########################################################## PUBLIC TASK API */

esp_err_t task::sampler::SamplerTask::start() {
	esp_err_t can_start     = ESP_OK;
	bool      transition_ok = false;
	if (!speed_qh) {
		ESP_LOGE(LOG_TAG, "No speed out queue!");
		can_start = ESP_ERR_INVALID_STATE;
	}
	if (!transition_handler) {
		can_start = ESP_ERR_INVALID_STATE;
	}
	if (ESP_OK == can_start && transition_handler) {
		transition_ok =
			transition_handler->update_state(SamplerState_e::SAMPLING);
		if ( !transition_ok ) {
			ESP_LOGE(LOG_TAG, "Start transition failed");
			return ESP_ERR_NOT_ALLOWED;
		}
	}
	return can_start;
}
esp_err_t task::sampler::SamplerTask::stop() {
	bool transition_ok =
		transition_handler->update_state(SamplerState_e::IDLE);
	return transition_ok? ESP_OK : ESP_ERR_INVALID_STATE;
}

task::sampler::SamplerTask::SamplerTask()
:	StateTask()
{
	_task_state_event_group_h = xEventGroupCreateStatic(
		&sampler_state_event_group
	);
	sampler_state_event_group_h = _task_state_event_group_h;
	xTaskCreate(
		sampler_task_fn,
		"sampler_task",
		2048 + 512,
		nullptr,
		3,
		&_frtos_task_h
	);
	xEventGroupWaitBits(
		_task_state_event_group_h,
		INIT_OK | SamplerState_e::ERROR,
		pdFALSE,
		pdFALSE,
		portMAX_DELAY
	);
	ESP_LOGI(LOG_TAG, "Task started!");
}

SamplerTask& task::sampler::SamplerTask::get_instance() {
	static SamplerTask singleton_sampler_task;
	return singleton_sampler_task;
}

void task::sampler::SamplerTask::set_params(const config_params& params) {
	speed_qh = params.speed_qh;
}

EventBits_t task::sampler::SamplerTask::get_state() {
	return xEventGroupGetBits(_task_state_event_group_h) & PUBLIC_STATE_MASK;
}

task::sampler::SamplerTask::~SamplerTask() {}
