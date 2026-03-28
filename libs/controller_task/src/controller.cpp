/**
 * @file controller.cpp
 * @author ACMAX (aavaloscorrales@gmail.com)
 * @brief
 * @version 0.1
 * @date 2026-03-21
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "controller_task.hpp"

#include "esp_log.h"
#include "esp_timer.h"

#include "controllers/pid_controller.hpp"

using namespace task;
using namespace task::controller;

const char LOG_TAG[] = "controller";

constexpr TickType_t CONTROLLER_TICK_TIME_ms = 2000;
constexpr int64_t    IDLE_TIME_us            = 3000000;
constexpr int64_t    WINDUP_PERIOD_us        = 6000000;
constexpr int64_t    CONTROL_PERIOD_us       = 6000000;
constexpr int64_t    WINDOWN_PERIOD_us       = 6000000;

static volatile EventGroupHandle_t controller_state_event_group_h = nullptr;

struct StateStruct_t {
	EventBits_t                      * current_state;
	Controller                       * controller;
	float                            * setpoint;
	StateSwitcher<ControllerState_e> * transition_handler;
};
struct TaskArgs_t {
	/* Task management variables */
	EventGroupHandle_t                *_controller_state_event_group_h;
	StateSwitcher<ControllerState_e> **_transition_handler;
	/* Runtime variables */
	Controller                       **_controller;
	/* Message interface variables */
	QueueHandle_t                     *_setpoint_qh;
	QueueHandle_t                     *_speed_qh;
	QueueHandle_t                     *_csignal_qh;
};

static void controller_task_fn(void *args);

static void handle_state(const StateStruct_t &state);
static void handle_error(const StateStruct_t &state);

static void idle_loop   (const StateStruct_t &state);
static void control_loop(const StateStruct_t &state);
static void windup_loop (const StateStruct_t &state);
static void windown_loop(const StateStruct_t &state);

static void controller_task_fn(void *args) {
	static StaticEventGroup_t controller_state_event_group;
	TaskArgs_t  *interface_attr     = (TaskArgs_t*)args;
	TickType_t   previous_wake_time = xTaskGetTickCount();
	PID         *controller         = nullptr;
	EventBits_t  current_state      = ControllerState_e::IDLE;
	float        setpoint           = 20.0f;
	StateSwitcher<ControllerState_e> *transition_handler = nullptr;

	if ( !controller_state_event_group_h ) {
		controller_state_event_group_h = xEventGroupCreateStatic(
			&controller_state_event_group
		);
	}
	transition_handler = new StateSwitcher<ControllerState_e>(
		controller_state_event_group_h,
		~ControllerState_e::ERROR
	);
	transition_handler->update_state(ControllerState_e::IDLE);

	std::function<float ()> error_func = [&setpoint] () -> float {
		return 0.0f - setpoint;
	};

	controller = new PID(error_func, 3.0f, 2.0f, 1.0f);
	controller->set_integrator_saturators(10.0);

	StateStruct_t state = {
		.current_state      = &current_state,
		.controller         =  controller,
		.setpoint           = &setpoint,
		.transition_handler =  transition_handler
	};

	*(interface_attr->_controller_state_event_group_h) = controller_state_event_group_h;
	*(interface_attr->_transition_handler            ) = transition_handler;
	*(interface_attr->_controller                    ) = controller;

	while (true) {
		current_state = xEventGroupGetBits(controller_state_event_group_h);

		if (current_state & ControllerState_e::ERROR) {
			handle_error(state);
		}
		else {
			handle_state(state);
		}

		(void)xTaskDelayUntil(
			&previous_wake_time,
			pdMS_TO_TICKS(CONTROLLER_TICK_TIME_ms)
		);
	}
}

void handle_state(const StateStruct_t &state) {
	switch (*state.current_state) {
		case ControllerState_e::IDLE:
			idle_loop   (state);
			break;
		case ControllerState_e::CONTROL:
			control_loop(state);
			break;
		case ControllerState_e::WINDUP:
			windup_loop (state);
			break;
		case ControllerState_e::WINDOWN:
			windown_loop(state);
			break;
		default:
			break;
	}
}

void handle_error(const StateStruct_t &state) {
	EventBits_t error_state =   *state.current_state
	                          & ~(ControllerState_e::ERROR);

	// Handle error
}

void idle_loop   (const StateStruct_t &state) {
	ESP_LOGI(LOG_TAG, "Idling");
	int64_t curr_time = esp_timer_get_time();
	if (   curr_time < WINDUP_PERIOD_us
		&& curr_time > IDLE_TIME_us
	) {
		state.transition_handler->update_state(ControllerState_e::WINDUP);
	}
}
void control_loop(const StateStruct_t &state) {
	static int64_t start_time = 0;
	int64_t curr_time  = esp_timer_get_time();

	if (!start_time) {
		start_time = curr_time;
	}
	else if ( curr_time > start_time+CONTROL_PERIOD_us ) {
		state.transition_handler->update_state(ControllerState_e::WINDOWN);
	}

	float control_signal = 0.0f;
	state.controller->loop();

	control_signal = state.controller->get_control_point().voltage;
	ESP_LOGI(LOG_TAG, "Control point is: %.3e", control_signal);
	ESP_LOGI(LOG_TAG, "Setpoint is     : %.3e", *state.setpoint);
}
void windup_loop (const StateStruct_t &state) {
	static int64_t start_time = 0;
	int64_t curr_time  = esp_timer_get_time();

	if (!start_time) {
		start_time = curr_time;
	}
	else if ( curr_time > start_time+WINDUP_PERIOD_us ) {
		state.transition_handler->update_state(ControllerState_e::CONTROL);
	}

	*(state.setpoint) += 3.0f;
	ESP_LOGI(LOG_TAG, "Windup setpoint: %.3e", *state.setpoint);
}
void windown_loop(const StateStruct_t &state) {
	static int64_t start_time = 0;
	int64_t curr_time  = esp_timer_get_time();

	if (!start_time) {
		start_time = curr_time;
	}
	else if ( curr_time > start_time+WINDOWN_PERIOD_us ) {
		state.transition_handler->update_state(ControllerState_e::IDLE);
	}

	*(state.setpoint) -= 3.0f;
	ESP_LOGI(LOG_TAG, "Windown setpoint: %.3e", *state.setpoint);
}

task::controller::ControllerTask::ControllerTask()
:
	/* Task management variables */
	_frtos_task_h                  (),
	_controller_state_event_group_h(),
	_transition_handler            (nullptr),
	/* Runtime variables */
	_controller (nullptr),
	/* Message interface variables */
	_setpoint_qh(),
	_speed_qh   (),
	_csignal_qh ()
{
	TaskArgs_t args {
		/* Task management variables */
		._controller_state_event_group_h = &_controller_state_event_group_h,
		._transition_handler             = &_transition_handler,
		/* Runtime variables */
		._controller = &_controller,
		/* Message interface variables */
		._setpoint_qh = &_setpoint_qh,
		._speed_qh    = &_speed_qh,
		._csignal_qh  = &_csignal_qh
	};
	xTaskCreate(
		controller_task_fn,
		"controller_task",
		2048,
		&args,
		3,
		&_frtos_task_h
	);
}

ControllerTask& task::controller::ControllerTask::get_instance() {
	static ControllerTask singleton_controller_task;
	return singleton_controller_task;
}

void task::controller::ControllerTask::set_params(const config_params& params) {
	_setpoint_qh = params.setpoint_qh;
	_speed_qh    = params.speed_qh;
	_csignal_qh  = params.control_signal_qh;
}
