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

#include "globals.hpp"

#include "esp_log.h"
#include "esp_timer.h"

#include "dc_plant.hpp"
#include "controller.hpp"
#include "controllers/pid_controller.hpp"
#include "controllers/open_loop_controller.hpp"
#include "controllers/ideal_control_law.hpp"
#include "windups/LinearWindup.hpp"
#include "windups/BezierWindup.hpp"
#include "winddowns/LinearWinddown.hpp"
#include "winddowns/BezierWinddown.hpp"
#include "sampler_task.hpp"
#include "apply_task.hpp"
#include "telemetry_task.hpp"

#define CONTROLLER_TYPE_PID   0
#define CONTROLLER_TYPE_OPEN  1
#define CONTROLLER_TYPE_IDEAL 2

#define CONTROLLER_TYPE CONTROLLER_TYPE_PID

using namespace task;
using namespace task::controller;

const char LOG_TAG[] = "controller";

constexpr TickType_t TELEMETRY_TICK_TIME_ms = SAMPLE_TIME_ms;
constexpr float      WINDUP_PERIOD_s         = 0.5f;
constexpr float      WINDOWN_PERIOD_s        = 0.5f;
constexpr int64_t    WATCHDOG_THRESH_us      = 1000;
constexpr int64_t    STOP_TIMEOUT_us         = TELEMETRY_TICK_TIME_ms*1000LL;
constexpr float      DEFAULT_SETPOINT        =  0.1f;//30.0f;
constexpr float      SP_DELTA_THRESHOLD      =  0.1f;

constexpr EventBits_t INIT_OK           = 0b1 << 11;
constexpr EventBits_t STATE_MASK        = ~(INIT_OK | ControllerState_e::ERROR);
constexpr EventBits_t PUBLIC_STATE_MASK = ~(INIT_OK);
constexpr EventBits_t CLEAR_BITS_MASK   = ~(0xff000000UL);

/* Task management variables */
static StaticEventGroup_t         controller_state_event_group;
static StaticEventGroup_t         controller_sync_event_group;
StateSwitcher<ControllerState_e> *transition_handler = nullptr;

static EventGroupHandle_t task_state_event_group_h;
static EventGroupHandle_t controller_sync_event_h;

/* Message interface variables */
static QueueHandle_t setpoint_qh = nullptr;
static QueueHandle_t csignal_qh  = nullptr;

/* Runtime variables */
static EventBits_t      current_task_state = 0;
static float            setpoint           = 0.0f;
static int64_t          windx_start        = 0L;
static Controller     * dc_controller      = nullptr;
static Windup         * current_windup   = nullptr;
static Winddown       * current_winddown = nullptr;
// static LinearWindup     default_windup  (WINDUP_PERIOD_s, 0.0f, DEFAULT_SETPOINT);
// static LinearWinddown   default_winddown(WINDOWN_PERIOD_s, DEFAULT_SETPOINT, 0.0f);
static BezierWindup   default_windup  (WINDUP_PERIOD_s, 0.0f, DEFAULT_SETPOINT);
static BezierWinddown default_winddown(WINDOWN_PERIOD_s, DEFAULT_SETPOINT, 0.0f);

static void control_task_fn(void *args);

static void handle_state();
static void handle_error();

static void idle_loop   ();
static void control_loop();
static void windup_loop ();
static void windown_loop();

static inline void control_tick();

static bool handle_transition(ControllerState_e from, ControllerState_e to);

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
	case ControllerState_e::STOPPING:
		return "STOPPING";
	default:
		return "UNKNOWN";
	}
}

static void control_task_fn(void *args) {
#if CONTROLLER_TYPE == CONTROLLER_TYPE_PID
	char controller_mem_space[sizeof(PID)                             ] = {0};
#endif
#if CONTROLLER_TYPE == CONTROLLER_TYPE_OPEN
	char controller_mem_space[sizeof(OpenLoop)                        ] = {0};
#endif
#if CONTROLLER_TYPE == CONTROLLER_TYPE_IDEAL
	char controller_mem_space[sizeof(IdealControlLaw)                 ] = {0};
#endif
	char trans_hand_mem_space[sizeof(StateSwitcher<ControllerState_e>)] = {0};
	TickType_t  previous_wake_time = xTaskGetTickCount();
	EventBits_t previous_state     = 0;
	EventBits_t state_delta        = 0;

	current_task_state   = ControllerState_e::IDLE;
	setpoint        = 0.0f;
	windx_start     = 0;

	(void)telemetry::TelemetryTask::get_instance();

	transition_handler = new (trans_hand_mem_space) StateSwitcher<ControllerState_e>(
		task_state_event_group_h,
		STATE_MASK
	);
	transition_handler->update_state(ControllerState_e::IDLE);

#if CONTROLLER_TYPE == CONTROLLER_TYPE_PID
	std::function<float (float)> error_func = [](float _setpoint) -> float {
		return _setpoint - Controller::read_speed_rad_s();
	};

	dc_controller = new (controller_mem_space) PID(error_func, 3.0f, 2.0f, 1.0f);
	((PID*)dc_controller)->set_integrator_saturators(10.0f);
#endif
#if CONTROLLER_TYPE == CONTROLLER_TYPE_OPEN
	dc_controller = new (controller_mem_space) OpenLoop();
#endif
#if CONTROLLER_TYPE == CONTROLLER_TYPE_IDEAL
	dc_controller = new (controller_mem_space) IdealControlLaw(
		3.1758f,
		0.4152f,
		0.0975f,
		0.4560f
	);
#endif

	transition_handler->set_trans_callback(handle_transition);

	xEventGroupSetBits(task_state_event_group_h, INIT_OK);

	while (true) {
		current_task_state  = xEventGroupGetBits(task_state_event_group_h);
		state_delta    = current_task_state ^ previous_state;
		(void)xEventGroupClearBits(controller_sync_event_h, CLEAR_BITS_MASK);
		(void)xEventGroupSetBits  (controller_sync_event_h, current_task_state);

		if (current_task_state & ControllerState_e::ERROR) {
			handle_error();
		}
		else {
			constexpr EventBits_t WINDX = ControllerState_e::WINDUP
			                            | ControllerState_e::WINDOWN
			                            | ControllerState_e::STOPPING;
			if (state_delta & current_task_state & WINDX ) {
				windx_start = esp_timer_get_time();
			}
			if (  state_delta
			    & current_task_state
			    & (ControllerState_e::WINDOWN | ControllerState_e::STOPPING)
			) {
				current_winddown->set_st_setpoint(setpoint);
				if (current_task_state & ControllerState_e::STOPPING) {
					current_winddown->set_en_setpoint(0.0f);
				}
			}
			handle_state();
		}

		previous_state = current_task_state;
		(void)xTaskDelayUntil(
			&previous_wake_time,
			pdMS_TO_TICKS(TELEMETRY_TICK_TIME_ms)
		);
	}
}

void handle_state() {
	ESP_LOGD(LOG_TAG, "state: %s", state_to_str(current_task_state & STATE_MASK));
	switch (current_task_state & STATE_MASK) {
		case ControllerState_e::IDLE:
			idle_loop   ();
			break;
		case ControllerState_e::CONTROL:
			control_loop();
			break;
		case ControllerState_e::WINDUP:
			windup_loop ();
			break;
		case ControllerState_e::WINDOWN:
		case ControllerState_e::STOPPING:
			windown_loop();
			break;
		default:
			break;
	}
}

void handle_error() {
	EventBits_t error_state =  current_task_state
	                          & ~(ControllerState_e::ERROR);

	// Handle error
}

inline void control_tick() {
	float control_signal       = 0.0f;

	(void)dc_controller->loop(setpoint);
	control_signal = dc_controller->get_control_point().voltage;
	ESP_LOGV(LOG_TAG, "Control point is: %.3e", control_signal);
	ESP_LOGV(LOG_TAG, "Setpoint is     : %.3e", setpoint);
	xQueueOverwrite(csignal_qh, &control_signal);
}

void idle_loop   () {
	ESP_LOGI(LOG_TAG, "Idling");
	if ( ESP_OK != sampler::SamplerTask::get_instance().stop() ) {
		ESP_LOGE(LOG_TAG, "Error while shutting down sampler");
	}
	(void)xEventGroupWaitBits(
		task_state_event_group_h,
		ControllerState_e::WINDUP | ControllerState_e::ERROR,
		pdFALSE,
		pdFALSE,
		portMAX_DELAY
	);
	/*:: WAIT END ::*/
	if ( ESP_OK != sampler::SamplerTask::get_instance().start() ) {
		ESP_LOGE(LOG_TAG, "Error starting sampler");
		xEventGroupSetBits(
			task_state_event_group_h,
			ControllerState_e::ERROR
		);
	}
	if ( ESP_OK != apply::ApplyTask::get_instance().start() ) {
		ESP_LOGE(LOG_TAG, "Error starting apply");
		xEventGroupSetBits(
			task_state_event_group_h,
			ControllerState_e::ERROR
		);
	}
	Windup   *controller_windup   = dc_controller->get_windup  ();
	Winddown *controller_winddown = dc_controller->get_winddown();
	current_windup   = controller_windup?   controller_windup   : &default_windup;
	current_winddown = controller_winddown? controller_winddown : &default_winddown;
	current_windup->set_st_setpoint(0.0f);
	dc_controller->setup();
}
void control_loop() {
	int64_t    curr_time       = esp_timer_get_time();
	float      dequed_setpoint = 0.0f;
	BaseType_t dequeue_success = pdTRUE;

	dequeue_success = xQueueReceive(
		setpoint_qh,
		&dequed_setpoint,
		0
	);
	if (pdFALSE == dequeue_success) {
		ESP_LOGV(
			LOG_TAG,
			"error receiving setpoint, using prev (%.3e)",
			setpoint
		);
	}
	else {
		const float setpoint_diff = setpoint - dequed_setpoint;
		if ( std::abs(setpoint_diff) > SP_DELTA_THRESHOLD ) {
			ESP_LOGI(LOG_TAG, "Big setpoint dif %e -> %e", setpoint, dequed_setpoint);
			if ( setpoint < dequed_setpoint ) {
				current_windup->set_st_setpoint(setpoint);
				current_windup->set_en_setpoint(dequed_setpoint);
				transition_handler->update_state(ControllerState_e::WINDUP);
			}
			else {
				current_winddown->set_st_setpoint(setpoint);
				current_winddown->set_en_setpoint(dequed_setpoint);
				transition_handler->update_state(ControllerState_e::WINDOWN);
			}
		}
		else {
			setpoint = dequed_setpoint;
		}
	}
	control_tick();
}
void windup_loop () {
	const int64_t curr_time    = esp_timer_get_time();
	const float   elapsed_time = (curr_time - windx_start)*1e-6f;

	if ( elapsed_time > current_windup->period() ) {
		transition_handler->update_state(ControllerState_e::CONTROL);
		setpoint = current_windup->get_en_setpoint();
	}
	else {
		setpoint = current_windup->step(elapsed_time);
	}

	ESP_LOGD(LOG_TAG, "Windup setpoint: %.3e", setpoint);
	control_tick();
}
void windown_loop() {
	const int64_t curr_time    = esp_timer_get_time();
	const float   elapsed_time = (curr_time - windx_start)*1e-6f;

	if ( elapsed_time > current_winddown->period() ) {
		switch ( current_task_state & STATE_MASK) {
			case ControllerState_e::WINDOWN:
				transition_handler->update_state(ControllerState_e::CONTROL);
				break;
			case ControllerState_e::STOPPING:
				transition_handler->update_state(ControllerState_e::IDLE);
				break;
		}
		setpoint = current_winddown->get_en_setpoint();
	}
	else {
		setpoint = current_winddown->step(elapsed_time);
	}

	ESP_LOGD(LOG_TAG, "Windown setpoint: %.3e", setpoint);
	control_tick();
}

/* ###################################################### TRANSITION HANDLING */

bool handle_transition(ControllerState_e from, ControllerState_e to) {
	ESP_LOGI(
		LOG_TAG,
		"Transitioning from %s to %s",
		state_to_str(from),
		state_to_str(to)
	);
	switch (to) {
	case IDLE:
		if (    WINDUP  == from
		     || WINDOWN == from
		) {
			return false;
		}
		break;
	case WINDUP:
		if (    WINDOWN  == from
		     || STOPPING == from
		) {
			return false;
		}
		break;
	case WINDOWN:
	case STOPPING:
		if (IDLE == from) {
			return false;
		}
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
	                                       	controller_sync_event_h
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
		current_exec_state     = xEventGroupGetBits(controller_sync_event_h);
	}
	return ESP_OK;
}

/* ########################################################## PUBLIC TASK API */

esp_err_t task::controller::ControllerTask::start() {
	esp_err_t can_start     = ESP_OK;
	bool      transition_ok = false;

	if (!setpoint_qh) {
		ESP_LOGE(LOG_TAG, "No setpoint out queue!");
		can_start = ESP_ERR_INVALID_STATE;
	}
	if (!csignal_qh) {
		ESP_LOGE(LOG_TAG, "No control signal out queue!");
		can_start = ESP_ERR_INVALID_STATE;
	}
	if (!transition_handler) {
		can_start = ESP_ERR_INVALID_STATE;
	}

	if (ESP_OK != wait_sync() ) {
		can_start = ESP_ERR_TIMEOUT;
	}
	if (ESP_OK == can_start && transition_handler) {
		transition_ok =
			transition_handler->update_state(ControllerState_e::WINDUP);
		if ( !transition_ok ) {
			ESP_LOGE(LOG_TAG, "Start transition failed");
			return ESP_ERR_NOT_ALLOWED;
		}
	}
	return can_start;
}
esp_err_t task::controller::ControllerTask::stop() {
	EventBits_t current_state = 0;

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
	if (current_state & ControllerState_e::STOPPING) {
		ESP_LOGE(LOG_TAG, "Already stopping!");
		return ESP_ERR_INVALID_STATE;
	}
	if (transition_handler) {
		transition_handler->update_state(ControllerState_e::STOPPING);
	}
	return ESP_OK;
}

task::controller::ControllerTask::ControllerTask()
:	StateTask()
{
	_task_state_event_group_h = xEventGroupCreateStatic(
		&controller_state_event_group
	);
	controller_sync_event_h = xEventGroupCreateStatic(
		&controller_sync_event_group
	);
	task_state_event_group_h = _task_state_event_group_h;
	controller_sync_event_h  = controller_sync_event_h;

	xTaskCreate(
		control_task_fn,
		"controller_task",
		2048+512,
		nullptr,
		2,
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
	setpoint_qh = params.setpoint_qh;
	csignal_qh  = params.control_signal_qh;
}

EventBits_t task::controller::ControllerTask::get_state() {
	return xEventGroupGetBits(controller_sync_event_h)
	       & PUBLIC_STATE_MASK;
}

esp_err_t task::controller::ControllerTask::wait_state(
	EventBits_t state, TickType_t timeout
) {
	EventBits_t obtained_status = 0;
	obtained_status = xEventGroupWaitBits(
		controller_sync_event_h,
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
