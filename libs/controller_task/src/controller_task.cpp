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
#include "controllers/pid_controller.hpp"
#include "controllers/open_loop_controller.hpp"
#include "controllers/fixed_sp_controller.hpp"
#include "encoder_task.hpp"
#include "apply_task.hpp"
#include "telemetry_task.hpp"

#define CONTROLLER_TYPE_PID  0
#define CONTROLLER_TYPE_OPEN 1
#define CONTROLLER_TYPE_FIXD 2

#define CONTROLLER_TYPE CONTROLLER_TYPE_OPEN

using namespace task;
using namespace task::controller;

const char LOG_TAG[] = "controller";

constexpr TickType_t TELEMETRY_TICK_TIME_ms = SAMPLE_TIME_ms;
constexpr int64_t    WINDUP_PERIOD_us        = 1500000;
constexpr int64_t    WINDOWN_PERIOD_us       = 1500000;
constexpr float      WINDUP_PERIODf_s        = WINDUP_PERIOD_us *1e-6f;
constexpr float      WINDOWN_PERIODf_s       = WINDOWN_PERIOD_us*1e-6f;
constexpr int64_t    WATCHDOG_THRESH_us      = 1000;
constexpr int64_t    STOP_TIMEOUT_us         = TELEMETRY_TICK_TIME_ms*1000LL;
constexpr float      DEFAULT_SETPOINT        = 30.0f;

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
static QueueHandle_t speed_qh    = nullptr;
static QueueHandle_t csignal_qh  = nullptr;

/* Runtime variables */
static EventBits_t   current_task_state = 0;
static float         setpoint           = 0.0f;
static float         windwn_setpoint    = 0.0f;
static int64_t       windx_start        = 0L;
static Controller  * dc_controller      = nullptr;
#if CONTROLLER_TYPE == CONTROLLER_TYPE_OPEN
static QueueHandle_t                      open_loop_voltage_qh = nullptr;
static DCPlant::EulerDCMotorModel const * motor_model          = nullptr;
#endif


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
#if CONTROLLER_TYPE == CONTROLLER_TYPE_FIXD
	constexpr int N_SETPOINTS = 3;
	char controller_mem_space[sizeof(FixedSPController<N_SETPOINTS>)  ] = {0};
#endif
	char trans_hand_mem_space[sizeof(StateSwitcher<ControllerState_e>)] = {0};
	TickType_t  previous_wake_time = xTaskGetTickCount();
	EventBits_t previous_state     = 0;
	EventBits_t state_delta        = 0;

	current_task_state   = ControllerState_e::IDLE;
	setpoint        = 0.0f;
	windwn_setpoint = 0.0f;
	windx_start     = 0;

	(void)telemetry::TelemetryTask::get_instance();

	transition_handler = new (trans_hand_mem_space) StateSwitcher<ControllerState_e>(
		task_state_event_group_h,
		STATE_MASK
	);
	transition_handler->update_state(ControllerState_e::IDLE);

#if CONTROLLER_TYPE == CONTROLLER_TYPE_PID
	std::function<float ()> error_func = [&setpoint] () -> float {
		return 0.0f - setpoint;
	};

	controller = new (controller_mem_space) PID(error_func, 3.0f, 2.0f, 1.0f);
	((PID*)controller)->set_integrator_saturators(10.0f);
#endif
#if CONTROLLER_TYPE == CONTROLLER_TYPE_OPEN
	const float voltage_setpoint = 30.0f;
	QueueHandle_t voltage_queue = xQueueCreate(1, sizeof(float));
	xQueueOverwrite(voltage_queue, &voltage_setpoint);
	dc_controller = new (controller_mem_space) OpenLoop(voltage_queue);
#endif
#if CONTROLLER_TYPE == CONTROLLER_TYPE_FIXD
	BzSetpoint_t setpoints[N_SETPOINTS] = {
		{.start_time=0.1f, .trans_time=0.5f, .setpoint=500.0f},
		{.start_time=1.7f, .trans_time=0.5f, .setpoint=250.0f},
		{.start_time=3.3f, .trans_time=0.5f, .setpoint=375.0f}
	};
	float rs[] = { 252.0f, -1050.0f, 1800.0f, -1575.0f, 700.0f, -126.0f};
	controller = new (controller_mem_space) FixedSPController<N_SETPOINTS>(
		setpoints, rs,
		3.1758f,
		0.4152f,
		0.0975f,
		0.4560f
	);
#endif

	transition_handler->set_trans_callback(handle_transition);

#if CONTROLLER_TYPE == CONTROLLER_TYPE_OPEN
	open_loop_voltage_qh = voltage_queue;
	motor_model          = &((OpenLoop*)dc_controller)->model();
#endif

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
			if (state_delta & current_task_state & ControllerState_e::WINDOWN) {
				windwn_setpoint = setpoint;
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
	ESP_LOGI(LOG_TAG, "state: %s", state_to_str(current_task_state & STATE_MASK));
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
	float speed                = 0.0f;
	BaseType_t dequeue_success = pdTRUE;

	dequeue_success = xQueuePeek(
		speed_qh,
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
#if CONTROLLER_TYPE == CONTROLLER_TYPE_OPEN
	xQueueOverwrite(open_loop_voltage_qh, &setpoint);
#endif
	dc_controller->loop();
	control_signal = dc_controller->get_control_point().voltage;
	ESP_LOGI(LOG_TAG, "Control point is: %.3e", control_signal);
	ESP_LOGI(LOG_TAG, "Setpoint is     : %.3e", setpoint);
	xQueueOverwrite(csignal_qh, &control_signal);
}

void idle_loop   () {
	ESP_LOGI(LOG_TAG, "Idling");
	if ( ESP_OK != encoder::EncoderTask::get_instance().stop() ) {
		ESP_LOGE(LOG_TAG, "Error while shutting down encoder");
	}
	(void)xEventGroupWaitBits(
		task_state_event_group_h,
		ControllerState_e::WINDUP | ControllerState_e::ERROR,
		pdFALSE,
		pdFALSE,
		portMAX_DELAY
	);
	/*:: WAIT END ::*/
	if ( ESP_OK != encoder::EncoderTask::get_instance().start() ) {
		ESP_LOGE(LOG_TAG, "Error starting encoder");
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
	dc_controller->setup();
	windx_start = esp_timer_get_time();
}
void control_loop() {
	int64_t    curr_time       = esp_timer_get_time();
	BaseType_t dequeue_success = pdTRUE;

	dequeue_success = xQueuePeek(
		setpoint_qh,
		&setpoint,
		pdMS_TO_TICKS(50)
	);
	if (pdFALSE == dequeue_success) {
		ESP_LOGW(
			LOG_TAG,
			"error receiving setpoint, using prev (%.3e)",
			setpoint
		);
	}
	control_tick();
}
void windup_loop () {
	const int64_t curr_time    = esp_timer_get_time();
	const float   elapsed_time = (curr_time - windx_start)*1e-6f;

	if ( curr_time > windx_start + WINDUP_PERIOD_us ) {
		transition_handler->update_state(ControllerState_e::CONTROL);
		setpoint = DEFAULT_SETPOINT;
	}
	else {
		setpoint = DEFAULT_SETPOINT*elapsed_time/WINDOWN_PERIODf_s;
	}

	ESP_LOGI(LOG_TAG, "Windup setpoint: %.3e", setpoint);
	control_tick();
}
void windown_loop() {
	const int64_t curr_time    = esp_timer_get_time();
	const float   elapsed_time = (curr_time - windx_start)*1e-6f;

	if ( curr_time > windx_start + WINDOWN_PERIOD_us ) {
		transition_handler->update_state(ControllerState_e::IDLE);
		setpoint = 0.0f;
	}
	else {
		setpoint = windwn_setpoint*(1.0f - elapsed_time/WINDOWN_PERIODf_s);
	}

	ESP_LOGI(LOG_TAG, "Windown setpoint: %.3e", setpoint);
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
		windx_start = esp_timer_get_time();
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
	if (!speed_qh) {
		ESP_LOGE(LOG_TAG, "No speed in queue!");
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
	if (transition_handler) {
		transition_handler->update_state(ControllerState_e::WINDOWN);
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
	speed_qh    = params.speed_qh;
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
