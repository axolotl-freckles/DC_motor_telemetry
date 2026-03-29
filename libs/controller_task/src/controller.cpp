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
constexpr int64_t    WINDUP_PERIOD_us        = 6000000;
constexpr int64_t    WINDOWN_PERIOD_us       = 6000000;

constexpr EventBits_t INIT_OK    = 0b1 << 11;
constexpr EventBits_t STATE_MASK = ~(INIT_OK | ControllerState_e::ERROR);

struct StateStruct_t {
	EventBits_t                      * current_state;
	Controller                       * controller;
	float                            * setpoint;
	StateSwitcher<ControllerState_e> * transition_handler;
	EventGroupHandle_t                 task_state_event_group_h;
	int64_t                          * windx_start;
};
struct TaskArgs_t {
	/* Task management variables */
	EventGroupHandle_t                *_task_state_event_group_h;
	StateSwitcher<ControllerState_e> **_transition_handler;
	/* Runtime variables */
	Controller                       **_controller;
	/* Message interface variables */
};
struct TransHandler_ft {
	Controller *controller;
	float      *setpoint;
	int64_t    *windx_start;

	bool operator () (ControllerState_e to, ControllerState_e from);
};
struct QueueHandles_t {
	QueueHandle_t setpoint_qh;
	QueueHandle_t speed_qh;
	QueueHandle_t csignal_qh;
};

static QueueHandles_t queues = {
	.setpoint_qh = nullptr,
	.speed_qh    = nullptr,
	.csignal_qh  = nullptr
};

static void controller_task_fn(void *args);

static void handle_state(const StateStruct_t &state);
static void handle_error(const StateStruct_t &state);

static void idle_loop   (const StateStruct_t &state);
static void control_loop(const StateStruct_t &state);
static void windup_loop (const StateStruct_t &state);
static void windown_loop(const StateStruct_t &state);

static inline void control_tick(const StateStruct_t &state) {
	float control_signal = 0.0f;

	state.controller->loop();
	control_signal = state.controller->get_control_point().voltage;
	ESP_LOGI(LOG_TAG, "Control point is: %.3e", control_signal);
	ESP_LOGI(LOG_TAG, "Setpoint is     : %.3e", *state.setpoint);
	xQueueOverwrite(queues.csignal_qh, &control_signal);
}

static void controller_task_fn(void *args) {
	EventGroupHandle_t  task_state_event_group_h;
	TaskArgs_t         *interface_attr     = (TaskArgs_t*)args;
	TickType_t          previous_wake_time = xTaskGetTickCount();
	PID                *controller         = nullptr;
	EventBits_t         current_state      = ControllerState_e::IDLE;
	float               setpoint           = 20.0f;
	int64_t             windx_start        = 0;
	StateSwitcher<ControllerState_e> *transition_handler = nullptr;

	task_state_event_group_h = *interface_attr->_task_state_event_group_h;

	transition_handler = new StateSwitcher<ControllerState_e>(
		*interface_attr->_task_state_event_group_h,
		STATE_MASK
	);
	transition_handler->update_state(ControllerState_e::IDLE);

	std::function<float ()> error_func = [&setpoint] () -> float {
		return 0.0f - setpoint;
	};

	controller = new PID(error_func, 3.0f, 2.0f, 1.0f);
	controller->set_integrator_saturators(10.0);

	TransHandler_ft handle_transition = {
		.controller  =  controller,
		.setpoint    = &setpoint,
		.windx_start = &windx_start,
	};
	transition_handler->set_trans_callback(handle_transition);

	StateStruct_t state = {
		.current_state            = &current_state,
		.controller               =  controller,
		.setpoint                 = &setpoint,
		.transition_handler       =  transition_handler,
		.task_state_event_group_h =  task_state_event_group_h,
		.windx_start              = &windx_start
	};

	*(interface_attr->_transition_handler) = transition_handler;
	*(interface_attr->_controller        ) = controller;

	xEventGroupSetBits(task_state_event_group_h, INIT_OK);

	while (true) {
		current_state = xEventGroupGetBits(task_state_event_group_h);

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
	switch (*state.current_state & STATE_MASK) {
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
	(void)xEventGroupWaitBits(
		state.task_state_event_group_h,
		ControllerState_e::WINDUP | ControllerState_e::ERROR,
		pdFALSE,
		pdFALSE,
		portMAX_DELAY
	);
	state.controller->setup();
	*state.windx_start = esp_timer_get_time();
}
void control_loop(const StateStruct_t &state) {
	int64_t    curr_time       = esp_timer_get_time();
	BaseType_t dequeue_success = pdTRUE;

	dequeue_success = xQueuePeek(
		queues.setpoint_qh,
		state.setpoint,
		pdMS_TO_TICKS(50)
	);
	if (pdFALSE == dequeue_success) {
		ESP_LOGW(
			LOG_TAG,
			"error receiving setpoint, using prev (%.3e)",
			*state.setpoint
		);
	}
	control_tick(state);
}
void windup_loop (const StateStruct_t &state) {
	int64_t start_time = *state.windx_start;
	int64_t curr_time  = esp_timer_get_time();

	if ( curr_time > start_time+WINDUP_PERIOD_us ) {
		state.transition_handler->update_state(ControllerState_e::CONTROL);
	}

	*state.setpoint += 3.0f;
	ESP_LOGI(LOG_TAG, "Windup setpoint: %.3e", *state.setpoint);
	control_tick(state);
}
void windown_loop(const StateStruct_t &state) {
	int64_t start_time = *state.windx_start;
	int64_t curr_time  = esp_timer_get_time();

	if ( curr_time > start_time+WINDOWN_PERIOD_us ) {
		state.transition_handler->update_state(ControllerState_e::IDLE);
	}

	*state.setpoint -= 3.0f;
	ESP_LOGI(LOG_TAG, "Windown setpoint: %.3e", *state.setpoint);
	control_tick(state);
}

bool TransHandler_ft::operator()(ControllerState_e from, ControllerState_e to) {
	ESP_LOGI(LOG_TAG, "Transitioning from %X to %X", from, to);
	switch (to) {
		case IDLE:
			if ( WINDUP == from ) {
				return false;
			}
			break;
		case WINDUP:
			if ( WINDOWN == from ) {
				return false;
			}
			break;
		case WINDOWN:
			if (IDLE == from) {
				return false;
			}
			*windx_start = esp_timer_get_time();
			break;
		default:
			break;
	}
	return true;
}

esp_err_t task::controller::ControllerTask::start() {
	esp_err_t can_start     = ESP_OK;
	bool      transition_ok = false;
	if (!_setpoint_qh) {
		ESP_LOGE(LOG_TAG, "No setpoint out queue!");
		can_start = ESP_ERR_INVALID_STATE;
	}
	if (!_speed_qh) {
		ESP_LOGE(LOG_TAG, "No speed in queue!");
		can_start = ESP_ERR_INVALID_STATE;
	}
	if (!_csignal_qh) {
		ESP_LOGE(LOG_TAG, "No control signal out queue!");
		can_start = ESP_ERR_INVALID_STATE;
	}
	if (!_transition_handler) {
		can_start = ESP_ERR_INVALID_STATE;
	}
	if (ESP_OK == can_start && _transition_handler) {
		transition_ok =
			_transition_handler->update_state(ControllerState_e::WINDUP);
		if ( !transition_ok ) {
			ESP_LOGE(LOG_TAG, "Start transition failed");
			return ESP_ERR_NOT_ALLOWED;
		}
	}
	return can_start;
}
esp_err_t task::controller::ControllerTask::stop() {
	if (_transition_handler) {
		_transition_handler->update_state(ControllerState_e::WINDOWN);
	}
	return ESP_OK;
}

task::controller::ControllerTask::ControllerTask()
:	StateTask()
	/* Task management variables */
	, _controller_state_event_group  ()
	, _transition_handler            (nullptr)
	/* Runtime variables */
	, _controller (nullptr)
	/* Message interface variables */
	, _setpoint_qh()
	, _speed_qh   ()
	, _csignal_qh ()
{
	TaskArgs_t args {
		/* Task management variables */
		._task_state_event_group_h = &_task_state_event_group_h,
		._transition_handler       = &_transition_handler,
		/* Runtime variables */
		._controller = &_controller,
		/* Message interface variables */
	};
	ESP_LOGI(LOG_TAG, "csignal container ptr: %p", &_csignal_qh);
	_task_state_event_group_h = xEventGroupCreateStatic(
		&_controller_state_event_group
	);
	xTaskCreate(
		controller_task_fn,
		"controller_task",
		2048 + 512,
		&args,
		3,
		&_frtos_task_h
	);
	xEventGroupWaitBits(
		_task_state_event_group_h,
		INIT_OK | ControllerState_e::ERROR,
		pdFALSE,
		pdFALSE,
		portMAX_DELAY
	);
	ESP_LOGI(LOG_TAG, "Task started!");
}

ControllerTask& task::controller::ControllerTask::get_instance() {
	static ControllerTask singleton_controller_task;
	return singleton_controller_task;
}

void task::controller::ControllerTask::set_params(const config_params& params) {
	_setpoint_qh = params.setpoint_qh;
	_speed_qh    = params.speed_qh;
	_csignal_qh  = params.control_signal_qh;

	queues.setpoint_qh = _setpoint_qh;
	queues.speed_qh    = _speed_qh;
	queues.csignal_qh  = _csignal_qh;
}

task::controller::ControllerTask::~ControllerTask() {}
