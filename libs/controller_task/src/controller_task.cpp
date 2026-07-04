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
#include <cstdlib>
#include "freertos/task.h"
#include "esp_heap_caps.h"

#include "controllers/pid_controller.hpp"
#include "encoder_task.hpp"

using namespace task;
using namespace task::controller;

const char LOG_TAG[] = "controller";

constexpr TickType_t CONTROLLER_TICK_TIME_ms = 2000;
constexpr int64_t    WINDUP_PERIOD_us        = 6000000;
constexpr int64_t    WINDOWN_PERIOD_us       = 6000000;
constexpr int64_t    WATCHDOG_THRESH_us      = 1000;
constexpr int64_t    STOP_TIMEOUT_us         = CONTROLLER_TICK_TIME_ms*1000LL;

constexpr EventBits_t INIT_OK           = 0b1 << 11;
constexpr EventBits_t STATE_MASK        = ~(INIT_OK | ControllerState_e::ERROR);
constexpr EventBits_t PUBLIC_STATE_MASK = ~(INIT_OK);
constexpr EventBits_t CLEAR_BITS_MASK   = ~(0xff000000UL);

struct StateStruct_t {
	EventBits_t                      * current_state             = nullptr;
	Controller                       * controller                = nullptr;
	float                            * setpoint                  = nullptr;
	StateSwitcher<ControllerState_e> * transition_handler        = nullptr;
	EventGroupHandle_t                 task_state_event_group_h  = nullptr;
	int64_t                          * windx_start               = nullptr;
};
struct TaskArgs_t {
	/* Task management variables */
	EventGroupHandle_t                *_task_state_event_group_h = nullptr;
	EventGroupHandle_t                *_controller_sync_event_h  = nullptr;
	StateSwitcher<ControllerState_e> **_transition_handler       = nullptr;
	/* Runtime variables */
	Controller                       **_controller               = nullptr;
	/* Message interface variables */
};
struct ControllerTransitionContext {
	Controller *controller  = nullptr;
	float      *setpoint    = nullptr;
	int64_t    *windx_start = nullptr;
};
static bool controller_transition_callback(
	ControllerState_e from,
	ControllerState_e to,
	void *context
);
struct QueueHandles_t {
	QueueHandle_t setpoint_qh = nullptr;
	QueueHandle_t speed_qh    = nullptr;
	QueueHandle_t csignal_qh  = nullptr;
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

static inline void control_tick(const StateStruct_t &state);

static inline const char * state_to_str(EventBits_t state_bits) {
	if (state_bits & ControllerState_e::ERROR) {
		return "ERROR";
	}
	switch (state_bits & STATE_MASK)
	{
	case ControllerState_e::IDLE:
		return "IDLE";
	case ControllerState_e::WINDUP:
		return "WINDUP";
	case ControllerState_e::WINDOWN:
		return "WINDOWN";
	case ControllerState_e::CONTROL:
		return "CONTROL";
	default:
		return "UNKNOWN";
	}
}

static void controller_task_fn(void *args) {
	void *controller_mem_space = std::malloc(sizeof(PID));
	void *trans_hand_mem_space = std::malloc(sizeof(StateSwitcher<ControllerState_e>));
	if (!controller_mem_space || !trans_hand_mem_space) {
		ESP_LOGE(LOG_TAG, "Failed to allocate controller buffers (heap low): %p %p", controller_mem_space, trans_hand_mem_space);
		if (controller_mem_space) std::free(controller_mem_space);
		if (trans_hand_mem_space) std::free(trans_hand_mem_space);
		vTaskDelay(pdMS_TO_TICKS(1000));
		vTaskDelete(NULL);
		return;
	}

	size_t free_heap = esp_get_free_heap_size();
	size_t min_free = heap_caps_get_minimum_free_size(MALLOC_CAP_DEFAULT);
	ESP_LOGI(LOG_TAG, "After malloc: free_heap=%u min_free=%u", (unsigned)free_heap, (unsigned)min_free);
	EventGroupHandle_t  task_state_event_group_h;
	EventGroupHandle_t  controller_sync_event_h;
	TaskArgs_t         *interface_attr     = (TaskArgs_t*)args;
	TickType_t          previous_wake_time = xTaskGetTickCount();
	PID                *controller         = nullptr;
	EventBits_t         current_state      = ControllerState_e::IDLE;
	float               setpoint           = 10.0f;
	int64_t             windx_start        = 0;
	StateSwitcher<ControllerState_e> *transition_handler = nullptr;

	task_state_event_group_h = *interface_attr->_task_state_event_group_h;
	controller_sync_event_h  = *interface_attr->_controller_sync_event_h;

	transition_handler = new (trans_hand_mem_space) StateSwitcher<ControllerState_e>(
		*interface_attr->_task_state_event_group_h,
		STATE_MASK
	);
	transition_handler->update_state(ControllerState_e::IDLE);

	// Error function: return a positive drive value based on setpoint
	// (no measurement available in this test harness). Using `setpoint`
	// makes the controller output positive so PWM duty follows windup.
	std::function<float ()> error_func = [&setpoint] () -> float {
		return setpoint;
	};

	controller = new (controller_mem_space) PID(error_func, 0.2f, 0.0f, 0.0f);
	controller->set_integrator_saturators(10.0);

	ControllerTransitionContext *transition_context = (ControllerTransitionContext*)std::malloc(sizeof(ControllerTransitionContext));
	if (!transition_context) {
		ESP_LOGE(LOG_TAG, "Failed to allocate transition context");
		vTaskDelay(pdMS_TO_TICKS(1000));
		vTaskDelete(NULL);
		return;
	}
	*transition_context = ControllerTransitionContext {
		.controller  = controller,
		.setpoint    = &setpoint,
		.windx_start = &windx_start,
	};
	transition_handler->set_trans_callback(
		controller_transition_callback,
		transition_context
	);

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
	std::free(interface_attr);

	xEventGroupSetBits(task_state_event_group_h, INIT_OK);

	while (true) {
		UBaseType_t highwater = uxTaskGetStackHighWaterMark(NULL);
		size_t free_heap_loop = esp_get_free_heap_size();
		ESP_LOGI(LOG_TAG, "controller_task stack high water mark: %u free_heap=%u", (unsigned)highwater, (unsigned)free_heap_loop);
		current_state = xEventGroupGetBits(task_state_event_group_h);
		(void)xEventGroupClearBits(controller_sync_event_h, CLEAR_BITS_MASK);
		(void)xEventGroupSetBits  (controller_sync_event_h, current_state);

		if (current_state & ControllerState_e::ERROR) {
			handle_error(state);
		}
		else {
			ESP_LOGI(LOG_TAG, "handle_state start: %s", state_to_str(current_state));
			handle_state(state);
			ESP_LOGI(LOG_TAG, "handle_state complete: %s", state_to_str(current_state));
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

inline void control_tick(const StateStruct_t &state) {
	float control_signal       = 0.0f;
	float speed                = 0.0f;
	BaseType_t dequeue_success = pdTRUE;

	dequeue_success = xQueuePeek(
		queues.speed_qh,
		&speed,
		pdMS_TO_TICKS(50)
	);

	if (pdFALSE == dequeue_success) {
		// TODO: Transition to error state
		ESP_LOGE(
			LOG_TAG,
			"error receiving speed, using prev (%.3e)",
			speed
		);
	}
	state.controller->loop();
	control_signal = state.controller->get_control_point().voltage;
	if (control_signal < 0.0f) {
		control_signal = 0.0f;
	} else if (control_signal > 24.0f) {
		control_signal = 24.0f;
		ESP_LOGW(LOG_TAG, "Control signal clamped to 24.0V");
	}
	ESP_LOGI(LOG_TAG, "Control point is: %.3e", control_signal);
	ESP_LOGI(LOG_TAG, "Setpoint is     : %.3e", *state.setpoint);
	xQueueOverwrite(queues.csignal_qh, &control_signal);
}

void idle_loop   (const StateStruct_t &state) {
	ESP_LOGI(LOG_TAG, "Idling");
	if ( ESP_OK != encoder::EncoderTask::get_instance().stop() ) {
		ESP_LOGE(LOG_TAG, "Error while shutting down encoder");
	}
	(void)xEventGroupWaitBits(
		state.task_state_event_group_h,
		ControllerState_e::WINDUP | ControllerState_e::ERROR,
		pdFALSE,
		pdFALSE,
		portMAX_DELAY
	);
	if ( ESP_OK != encoder::EncoderTask::get_instance().start() ) {
		ESP_LOGE(LOG_TAG, "Error starting encoder");
		xEventGroupSetBits(
			state.task_state_event_group_h,
			ControllerState_e::ERROR
		);
	}
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

	UBaseType_t highwater = uxTaskGetStackHighWaterMark(NULL);
	ESP_LOGI(LOG_TAG, "windup_loop stack high water mark: %u", (unsigned)highwater);

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

/* ###################################################### TRANSITION HANDLING */

static bool controller_transition_callback(
	ControllerState_e from,
	ControllerState_e to,
	void *context
) {
	ControllerTransitionContext *ctx = static_cast<ControllerTransitionContext*>(context);
	if (!ctx) {
		ESP_LOGE(LOG_TAG, "controller_transition_callback missing context");
		return false;
	}
	ESP_LOGI(
		LOG_TAG,
		"Transitioning from %s to %s",
		state_to_str(from),
		state_to_str(to)
	);
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
		*ctx->windx_start = esp_timer_get_time();
		break;
	default:
		break;
	}
	return true;
}

esp_err_t task::controller::ControllerTask::wait_sync() {
	EventBits_t   current_signaled_state = xEventGroupGetBits(
	                                       	_task_state_event_group_h
	                                       );
	EventBits_t   current_exec_state     = xEventGroupGetBits(
	                                       	_controller_sync_event_group_h
	                                       );
	int64_t const trans_attempt_st       = esp_timer_get_time();
	int64_t       trans_attempt_curr     = 0;

	while (current_signaled_state != current_exec_state) {
		trans_attempt_curr = esp_timer_get_time();
		if (trans_attempt_curr - trans_attempt_st > STOP_TIMEOUT_us) {
			return ESP_ERR_TIMEOUT;
		}
		if (trans_attempt_curr - trans_attempt_st > WATCHDOG_THRESH_us) {
			vTaskDelay(1);
		}
		current_signaled_state = xEventGroupGetBits(_task_state_event_group_h);
		current_exec_state     = xEventGroupGetBits(_controller_sync_event_group_h);
	}
	return ESP_OK;
}

/* ########################################################## PUBLIC TASK API */

esp_err_t task::controller::ControllerTask::start() {
	esp_err_t can_start     = ESP_OK;
	bool      transition_ok = false;

	if (!queues.setpoint_qh) {
		ESP_LOGE(LOG_TAG, "No setpoint out queue!");
		can_start = ESP_ERR_INVALID_STATE;
	}
	if (!queues.speed_qh) {
		ESP_LOGE(LOG_TAG, "No speed in queue!");
		can_start = ESP_ERR_INVALID_STATE;
	}
	if (!queues.csignal_qh) {
		ESP_LOGE(LOG_TAG, "No control signal out queue!");
		can_start = ESP_ERR_INVALID_STATE;
	}
	if (!_transition_handler) {
		can_start = ESP_ERR_INVALID_STATE;
	}

	if (ESP_OK != wait_sync() ) {
		can_start = ESP_ERR_TIMEOUT;
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
	EventBits_t   current_state = 0;

	if (ESP_OK != wait_sync() ) {
		return ESP_ERR_TIMEOUT;
	}
	current_state = get_state();
	if (current_state & ControllerState_e::ERROR) {
		return ESP_ERR_NOT_ALLOWED;
	}
	if (current_state & ControllerState_e::IDLE) {
		ESP_LOGW(LOG_TAG, "Already stopped!");
		return ESP_OK;
	}
	if (current_state & ControllerState_e::WINDOWN) {
		ESP_LOGE(LOG_TAG, "On windown!");
		return ESP_ERR_INVALID_STATE;
	}
	if (_transition_handler) {
		_transition_handler->update_state(ControllerState_e::WINDOWN);
	}
	return ESP_OK;
}

task::controller::ControllerTask::ControllerTask()
:	StateTask()
	/* Task management variables */
	, _controller_state_event_group  ()
	, _controller_sync_event_group   ()
	, _controller_sync_event_group_h (nullptr)
	, _transition_handler            (nullptr)
	/* Runtime variables */
	, _controller (nullptr)
	/* Message interface variables */
	, _setpoint_qh()
	, _speed_qh   ()
	, _csignal_qh ()
{
	TaskArgs_t *args = (TaskArgs_t*)std::malloc(sizeof(TaskArgs_t));
	if (!args) {
		ESP_LOGE(LOG_TAG, "Failed to allocate task args");
		return;
	}
	*args = TaskArgs_t {
		/* Task management variables */
		._task_state_event_group_h = &_task_state_event_group_h,
		._controller_sync_event_h  = &_controller_sync_event_group_h,
		._transition_handler       = &_transition_handler,
		/* Runtime variables */
		._controller = &_controller,
		/* Message interface variables */
	};
	_task_state_event_group_h = xEventGroupCreateStatic(
		&_controller_state_event_group
	);
	_controller_sync_event_group_h = xEventGroupCreateStatic(
		&_controller_sync_event_group
	);
	xTaskCreate(
		controller_task_fn,
		"controller_task",
		16384,
		args,
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

EventBits_t task::controller::ControllerTask::get_state() {
	return xEventGroupGetBits(_controller_sync_event_group_h)
	       & PUBLIC_STATE_MASK;
}

esp_err_t task::controller::ControllerTask::wait_state(
	EventBits_t state, TickType_t timeout
) {
	EventBits_t obtained_status = 0;
	obtained_status = xEventGroupWaitBits(
		_controller_sync_event_group_h,
		state | ControllerState_e::ERROR,
		pdFALSE, pdFALSE,
		timeout
	);

	if ( state & ControllerState_e::ERROR ) {
		if ( obtained_status & ControllerState_e::ERROR ) {
			return ESP_OK;
		}
		else {
			return ESP_ERR_TIMEOUT;
		}
	}
	else if ( obtained_status & ControllerState_e::ERROR ) {
		return ESP_ERR_INVALID_RESPONSE;
	}

	state           &= PUBLIC_STATE_MASK;
	obtained_status &= state;
	if ( !(obtained_status ^ state) ) {
		return ESP_ERR_TIMEOUT;
	}

	return ESP_OK;
}

task::controller::ControllerTask::~ControllerTask() {}
