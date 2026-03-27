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

#include "tasks.hpp"

#include <cstdlib>

#include "esp_log.h"
#include "esp_timer.h"

using namespace task;
using namespace task::encoder;

const char LOG_TAG[] = "encoder";

constexpr TickType_t ENCODER_TICK_TIME_ms = 2000;

static volatile EventGroupHandle_t encoder_state_event_group_h = nullptr;

struct StateStruct_t {
	EventBits_t                   * current_state;
	StateSwitcher<EncoderState_e> * transition_handler;
};

static void handle_state(const StateStruct_t &state);
static void handle_error(const StateStruct_t &state);

static void idle_loop  (const StateStruct_t &state);
static void sample_loop(const StateStruct_t &state);

void task::encoder_task(void *args) {
	static StaticEventGroup_t encoder_state_event_group;
	TickType_t  previous_wake_time = xTaskGetTickCount();
	EventBits_t current_state      = EncoderState_e::IDLE;
	StateSwitcher<EncoderState_e> *transition_handler = nullptr;

	if ( !encoder_state_event_group_h ) {
		encoder_state_event_group_h = xEventGroupCreateStatic(
			&encoder_state_event_group
		);
	}
	transition_handler = new StateSwitcher<EncoderState_e>(
		encoder_state_event_group_h,
		~EncoderState_e::ERROR
	);
	transition_handler->update_state(EncoderState_e::IDLE);

	StateStruct_t state = {
		.current_state      = &current_state,
		.transition_handler =  transition_handler
	};

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
	switch (*state.current_state) {
		case EncoderState_e::IDLE:
			idle_loop  (state);
			break;
		case EncoderState_e::SAMPLING:
			sample_loop(state);
			break;
	}
}

void idle_loop  (const StateStruct_t &state) {
	return;
}
void sample_loop(const StateStruct_t &state) {
	float speed_sample = 200.0f + std::rand()%100;
	speed_sample /= 3.0f;

	ESP_LOGI(LOG_TAG, "speed: %.3e", speed_sample);
}
