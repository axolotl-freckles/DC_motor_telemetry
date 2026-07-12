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

#include "globals.hpp"
#include "dc_plant.hpp"
#include <cmath>

using namespace task;
using namespace task::sampler;
using namespace DCPlant;

const char LOG_TAG[] = "sampler";

constexpr TickType_t ENCODER_TICK_TIME_ms = SAMPLE_TIME_ms;

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

static Encoder encoder((gpio_num_t)4, (gpio_num_t)5, 1600);
static EulerDCMotorModel mock_dc_motor(SAMPLE_PARAMS, MODEL_SIM_TIME_s);
static DCMotorObserver   observer(SAMPLE_PARAMS, SAMPLE_OBS_PRMS, MODEL_SIM_TIME_s);
static uint64_t          last_interpolation_us = 0L;
static float             last_voltage          = 0.0f;

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

static void interpolate_simulation();

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

	encoder.init();

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
	const float speed_sample = encoder.getRpm() * 2.0f * M_PI / 60.0f;
	ESP_LOGV(LOG_TAG, "speed: %.3e", speed_sample);
}

static void interpolate_simulation() {
	const uint64_t now_us = esp_timer_get_time();
	const uint64_t elapsed_time_ms = (now_us-last_interpolation_us)/1000L;
	const uint32_t missing_step_n  = (uint32_t)elapsed_time_ms/MODEL_SIM_TIME_ms;

	for (uint32_t i=0; i<missing_step_n; i++) {
		observer     .step(last_voltage, mock_dc_motor.state());
		mock_dc_motor.step(last_voltage, 1.0f);
	}
	last_interpolation_us = now_us;
}

/* ###################################################### TRANSITION HANDLING */

bool handle_transition(SamplerState_e from, SamplerState_e to) {
	ESP_LOGI(
		LOG_TAG,
		"Transitioning from %s to %s",
		state_to_str(from),
		state_to_str(to)
	);
	if ( SamplerState_e::SAMPLING == to ) {
		last_interpolation_us = esp_timer_get_time();
		last_voltage          = 0.0f;
		observer     .reset();
		mock_dc_motor.reset();
	}
	return true;
}

/* ########################################################## PUBLIC TASK API */

esp_err_t task::sampler::SamplerTask::start() {
	esp_err_t can_start     = ESP_OK;
	bool      transition_ok = false;
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

void task::sampler::SamplerTask::set_params(const config_params& params) { }

EventBits_t task::sampler::SamplerTask::get_state() {
	return xEventGroupGetBits(_task_state_event_group_h) & PUBLIC_STATE_MASK;
}

float task::sampler::SamplerTask::current_w()  {
	interpolate_simulation();
	//return mock_dc_motor.state().w_rad_s;

	return encoder.getRpm() * 2.0f * M_PI / 60.0f;
}
float task::sampler::SamplerTask::current_TL() {
	interpolate_simulation();
	return observer.estimated_load();
}
float task::sampler::SamplerTask::current_TI() {
	return observer.state().I_amp;
}
float task::sampler::SamplerTask::current_Volt() {
	return last_voltage;
}
float task::sampler::SamplerTask::estimated_load() {
	return observer.estimated_load();
}

const Encoder &task::sampler::SamplerTask::get_encoder() const {
	return encoder;
}

void task::sampler::SamplerTask::set_applied_voltage(float applied_voltage) {
	interpolate_simulation();
	last_voltage = applied_voltage;
}

task::sampler::SamplerTask::~SamplerTask() {}
