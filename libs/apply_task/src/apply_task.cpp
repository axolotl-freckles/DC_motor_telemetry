/**
 * @file telemetry_task.cpp
 * @author ACMAX (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2026-07-01
 *
 * @copyright Copyright (c) 2026
 *
 */
#include "apply_task.hpp"

#include "esp_log.h"
#include "driver/ledc.h"

using namespace task::apply;

const char LOG_TAG[] = "apply";

constexpr int      PWM_RESOLUTION   = 9;
constexpr uint32_t PWM_MAX_VAL      = (1<<PWM_RESOLUTION) - 1;
constexpr uint32_t PWM_FREQUENCY_Hz = 100000;
constexpr int      HIGH_GPIO        = 18;
constexpr int      LOW_GPIO         = 21;
constexpr ledc_channel_t HIGH_CHANNEL = (ledc_channel_t)1;
constexpr ledc_channel_t LOW_CHANNEL  = (ledc_channel_t)2;

struct StateStruct_t {
	EventBits_t current_state = 0;
};

const ledc_timer_config_t   pwm_timer_config = {
	.speed_mode      = LEDC_HIGH_SPEED_MODE,
	.duty_resolution = (ledc_timer_bit_t)PWM_RESOLUTION,
	.timer_num       = (ledc_timer_t)0,
	.freq_hz         = PWM_FREQUENCY_Hz,
	.clk_cfg         = LEDC_USE_APB_CLK,
	.deconfigure     = false
};

static QueueHandle_t      voltage_qh     = NULL;
static EventGroupHandle_t state_event_gh = NULL;

static void apply_task_fn (void* args);

static void handle_state(StateStruct_t &state);
static void handle_error(StateStruct_t &state);

static void idle_loop (StateStruct_t &state);
static void apply_loop(StateStruct_t &state);

void apply_task_fn (void* raw_args) {
	ApplyTask::ApplyTask_args *args = (ApplyTask::ApplyTask_args*)raw_args;
	StateStruct_t state;
	EventBits_t   current_state  = ApplyState_e::IDLE;
	EventBits_t   previous_state = 0;
	EventBits_t   state_delta    = 0;

	state_event_gh = args->state_event_group_h;

	xEventGroupSetBits(state_event_gh, ApplyState_e::IDLE);

	while (true) {
		current_state  = xEventGroupGetBits(state_event_gh);
		state_delta    = current_state ^ previous_state;
		// (void)xEventGroupClearBits(controller_sync_event_h, CLEAR_BITS_MASK);
		// (void)xEventGroupSetBits  (controller_sync_event_h, current_state);
		state.current_state = current_state;

		if (current_state & ApplyState_e::ERROR) {
			handle_error(state);
		}
		else {
			handle_state(state);
		}

		vTaskDelay(10);
	}

}

void handle_state(StateStruct_t &state) {
	switch (state.current_state) {
		case ApplyState_e::IDLE:
			idle_loop (state);
			break;
		case ApplyState_e::APPLYING:
			apply_loop(state);
			break;
		default:
			break;
	}
}
void handle_error(StateStruct_t &state) { }

void idle_loop(StateStruct_t &state) {
	ESP_LOGI(LOG_TAG, "Idling");
	(void)xEventGroupWaitBits(
		state_event_gh,
		ApplyState_e::APPLYING | ApplyState_e::ERROR,
		pdFALSE,
		pdFALSE,
		portMAX_DELAY
	);
}

void apply_loop(StateStruct_t &state) {
	EventBits_t curr_state = 0;
	float       voltage    = 0.0f;
	float       dutycycle  = 0.0f;
	(void)xQueueReceive(voltage_qh, &voltage, portMAX_DELAY);
	curr_state = xEventGroupGetBits(state_event_gh);
	if (curr_state & ~ApplyState_e::APPLYING) {
		return;
	}

	dutycycle = voltage / (voltage + 24.0f);
	dutycycle = std::max(dutycycle, 0.1f);
	dutycycle = std::min(dutycycle, 0.9f);
	ledc_set_duty_and_update(LEDC_HIGH_SPEED_MODE, HIGH_CHANNEL, dutycycle*PWM_MAX_VAL , 0);
	ledc_set_duty_and_update(LEDC_HIGH_SPEED_MODE, HIGH_CHANNEL, (1-dutycycle)*PWM_MAX_VAL , 0);
}

/* ###################################################### TRANSITION HANDLING */

/* ########################################################## PUBLIC TASK API */

QueueHandle_t task::apply::ApplyTask::createQueue(UBaseType_t len) {
	return xQueueCreate(len, sizeof(float));
}

esp_err_t task::apply::ApplyTask::start() {
	esp_err_t   can_start     = ESP_OK;
	EventBits_t curr_state    = xEventGroupGetBits(_task_state_event_group_h);
	bool        transition_ok = false;
	if (!voltage_qh) {
		ESP_LOGE(LOG_TAG, "No voltage queue handler!");
		can_start = ESP_ERR_INVALID_STATE;
	}
	if (curr_state & ApplyState_e::ERROR) {
		ESP_LOGE(LOG_TAG, "Apply in error state!");
		can_start = ESP_ERR_INVALID_STATE;
	}
	if (ESP_OK == can_start) {
		xEventGroupClearBits(_task_state_event_group_h, curr_state);
		xEventGroupSetBits  (_task_state_event_group_h, ApplyState_e::IDLE);
	}
	return can_start;
}
esp_err_t task::apply::ApplyTask::stop() {
	EventBits_t curr_state    = xEventGroupGetBits(_task_state_event_group_h);
	if (curr_state & ApplyState_e::ERROR) {
		ESP_LOGE(LOG_TAG, "Apply in error state!");
		return ESP_ERR_INVALID_STATE;
	}
	xEventGroupClearBits(_task_state_event_group_h, curr_state);
	xEventGroupSetBits  (_task_state_event_group_h, ApplyState_e::IDLE);
	return true;
}

void task::apply::ApplyTask::set_params(const config_params& params) {
	_config.voltage_queue_h = params.voltage_queue_h;
	voltage_qh              = params.voltage_queue_h;
}

ApplyTask& task::apply::ApplyTask::get_instance() {
	static task::apply::ApplyTask apply_task;
	return apply_task;
}

task::apply::ApplyTask::ApplyTask () : task::StateTask() {
	esp_err_t error_code = ESP_OK;

	_task_state_event_group_h = xEventGroupCreateStatic (
		&_apply_state_event_group
	);
	_task_fn_args.state_event_group_h = _task_state_event_group_h;

	ESP_LOGI(LOG_TAG, "Configuring PWM timer...");
	uint32_t suitable_res = ledc_find_suitable_duty_resolution(
		APB_CLK_FREQ, PWM_FREQUENCY_Hz
	);
	error_code = ESP_ERROR_CHECK_WITHOUT_ABORT(
		ledc_timer_config(&pwm_timer_config)
	);
	if (suitable_res > PWM_RESOLUTION) {
		ESP_LOGW(LOG_TAG,
			"You can increase the resolution to %ld",
			suitable_res
		);
	}
	if (error_code != ESP_OK) {
		ESP_LOGW(LOG_TAG,
			"With a frequency of %ldHz, a resolution of %ld is needed",
			PWM_FREQUENCY_Hz, suitable_res
		);
		xEventGroupSetBits(_task_state_event_group_h, ApplyState_e::ERROR);
	}
	ESP_LOGI(LOG_TAG, "PWM timer configured!");
	ledc_channel_config_t channel_high_config = {
		.gpio_num   = 0,
		.speed_mode = LEDC_HIGH_SPEED_MODE,
		.channel    = (ledc_channel_t)0,
		.intr_type  = LEDC_INTR_DISABLE,
		.timer_sel  = (ledc_timer_t)0,
		.duty       = 0x0F,
		.hpoint     = 0,
		.flags = {.output_invert = 0}
	};
	ledc_channel_config_t channel_low_config = channel_high_config;
	channel_high_config.gpio_num = HIGH_GPIO;
	channel_high_config.channel  = HIGH_CHANNEL;
	channel_low_config .gpio_num = LOW_GPIO;
	channel_low_config .channel  = LOW_CHANNEL;
	channel_high_config.flags.output_invert = 1;

	error_code = ESP_ERROR_CHECK_WITHOUT_ABORT(
		ledc_channel_config(&channel_high_config)
	);
	if (error_code != ESP_OK) {
		ESP_LOGE( LOG_TAG,
			"Error configuring high phase, ERRCODE:\n%s",
			esp_err_to_name(error_code));
		xEventGroupSetBits(_task_state_event_group_h, ApplyState_e::ERROR);
	}
	error_code = ESP_ERROR_CHECK_WITHOUT_ABORT(
		ledc_channel_config(&channel_low_config)
	);
	if (error_code != ESP_OK) {
		ESP_LOGE( LOG_TAG,
			"Error configuring high phase, ERRCODE:\n%s",
			esp_err_to_name(error_code));
		xEventGroupSetBits(_task_state_event_group_h, ApplyState_e::ERROR);
	}

	xTaskCreate (
		apply_task_fn,
		"apply_task",
		1024,
		&_task_fn_args,
		3,
		&_frtos_task_h
	);
	xEventGroupWaitBits (
		_task_state_event_group_h,
		ApplyState_e::ERROR | ApplyState_e::IDLE,
		pdFALSE,
		pdFALSE,
		portMAX_DELAY
	);
	ESP_LOGI(LOG_TAG, "Task started!");
};

task::apply::ApplyTask::~ApplyTask() {}
