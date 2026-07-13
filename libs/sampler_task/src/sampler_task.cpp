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

constexpr TickType_t ENCODER_TICK_TIME_ms = MODEL_SIM_TIME_ms;

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
static EulerDCMotorModel  mock_dc_motor(SAMPLE_PARAMS, MODEL_SIM_TIME_s);
static DCMotorObserver_64 observer(SAMPLE_PARAMS, SAMPLE_OBS_PRMS, MODEL_SIM_TIME_s);
static uint64_t           last_interpolation_us = 0L;

static esp_timer_handle_t sampler_timer_handle;

static volatile int64_t last_voltage_sh        = 0;
static volatile int64_t estimated_load_Nm_sh   = 0;
static volatile int64_t estimated_current_A_sh = 0;

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

static void IRAM_ATTR take_sample(void* args);

static void sampler_task_fn(void* args);

static void handle_state(const StateStruct_t &state);
static void handle_error(const StateStruct_t &state);

static void idle_loop  (const StateStruct_t &state);
static void sample_loop(const StateStruct_t &state);

static bool handle_transition(SamplerState_e from, SamplerState_e to);

static void interpolate_simulation();

void take_sample(void* args) {
	int64_t velocity = encoder.getW_rads_i();
	DCMotorObserver_64::EstimationResults res = observer.step(
		last_voltage_sh,
		velocity
	);
	estimated_load_Nm_sh   = res.load_Nm_sh;
	estimated_current_A_sh = res.I_amp_sh;
}

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
			pdMS_TO_TICKS(SAMPLE_TIME_ms)
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

// static void interpolate_simulation() {
// 	const uint64_t now_us = esp_timer_get_time();
// 	const uint64_t elapsed_time_ms = (now_us-last_interpolation_us)/1000L;
// 	const uint32_t missing_step_n  = (uint32_t)elapsed_time_ms/MODEL_SIM_TIME_ms;
// 	const float last_voltage = DCMotorObserver_64::from_repr(last_voltage_sh);

// 	for (uint32_t i=0; i<missing_step_n; i++) {
// 		observer     .step(last_voltage, mock_dc_motor.state());
// 		mock_dc_motor.step(last_voltage, 1.0f);
// 	}
// 	last_interpolation_us = now_us;
// }

/* ###################################################### TRANSITION HANDLING */

bool handle_transition(SamplerState_e from, SamplerState_e to) {
	esp_err_t error_code = ESP_OK;
	ESP_LOGI(
		LOG_TAG,
		"Transitioning from %s to %s",
		state_to_str(from),
		state_to_str(to)
	);
	if (
		     SamplerState_e::IDLE     == from
		  && SamplerState_e::SAMPLING ==   to
		) {
		observer.reset();
		error_code = ESP_ERROR_CHECK_WITHOUT_ABORT( esp_timer_start_periodic(
			sampler_timer_handle, MODEL_SIM_TIME_us
		));
		if (error_code != ESP_OK) {
			ESP_LOGE( LOG_TAG,
				"error starting phases: %s",
				esp_err_to_name(error_code)
			);
			return false;
		}
	}
	if (
		   SamplerState_e::SAMPLING == from
		&& SamplerState_e::IDLE     ==   to
	) {
		error_code = ESP_ERROR_CHECK_WITHOUT_ABORT(esp_timer_stop(sampler_timer_handle));
		if (error_code != ESP_OK) {
			ESP_LOGE( LOG_TAG,
				"error stopping sampler: %s",
				esp_err_to_name(error_code)
			);
			return false;
		}
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

	esp_timer_create_args_t sampler_timer_cfg {
		.callback              = take_sample,
		.arg                   = (void*)nullptr,
		.dispatch_method       = ESP_TIMER_ISR,
		.name                  = "SAMPLER",
		.skip_unhandled_events = false
	};
	esp_err_t error_code = ESP_ERROR_CHECK_WITHOUT_ABORT(esp_timer_create(
		&sampler_timer_cfg,
		&sampler_timer_handle
	));
	if (error_code != ESP_OK) {
		ESP_LOGE(LOG_TAG, "Unable to create timer interrupt");
		xEventGroupSetBits(_task_state_event_group_h, SamplerState_e::ERROR);
		return;
	}

	xTaskCreate(
		sampler_task_fn,
		"sampler_task",
		2048 + 1024,
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
	return DCMotorObserver_64::from_repr(encoder.getW_rads_i());
}
float task::sampler::SamplerTask::current_TL() {
	return DCMotorObserver_64::from_repr(estimated_load_Nm_sh);
}
float task::sampler::SamplerTask::current_TI() {
	return DCMotorObserver_64::from_repr(estimated_current_A_sh);
}
float task::sampler::SamplerTask::current_Volt() {
	return DCMotorObserver_64::from_repr(last_voltage_sh);
}

const Encoder &task::sampler::SamplerTask::get_encoder() const {
	return encoder;
}

void task::sampler::SamplerTask::set_applied_voltage(float applied_voltage) {
	last_voltage_sh = DCMotorObserver_64::to_repr(applied_voltage);
}

task::sampler::SamplerTask::~SamplerTask() {}
