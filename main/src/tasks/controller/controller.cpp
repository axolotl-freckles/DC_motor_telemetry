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

#include "tasks.hpp"

#include "controllers/pid_controller.hpp"

using namespace task;
using namespace task::controller;

constexpr TickType_t CONTROLLER_TICK_TIME_ms = 2000;

static volatile EventGroupHandle_t controller_state_event_group_h = nullptr;

static void handle_state(EventBits_t &current_state, Controller *controller);
static void handle_error(EventBits_t &current_state, Controller *controller);

static void control_loop(Controller *controller);
static void windup_loop(Controller *controller);
static void windown_loop(Controller *controller);

void task::controller_task(void *args) {
	static StaticEventGroup_t controller_state_event_group;
	TickType_t   previous_wake_time = xTaskGetTickCount();
	PID         *controller         = nullptr;
	EventBits_t  current_state      = ControllerState_e::IDLE;
	float        setpoint           = 20.0f;

	controller_state_event_group_h = xEventGroupCreateStatic(
		&controller_state_event_group
	);

	std::function<float ()> error_func = [&setpoint] () -> float {
		return 0.0f - setpoint;
	};

	controller = new PID(error_func, 3.0f, 2.0f, 1.0f);
	controller->set_integrator_saturators(10.0);

	while (true) {
		current_state = xEventGroupGetBits(controller_state_event_group_h);

		if (current_state & ControllerState_e::ERROR) {
			handle_error(current_state, controller);
		}
		else {
			handle_state(current_state, controller);
		}

		(void)xTaskDelayUntil(
			&previous_wake_time,
			pdMS_TO_TICKS(CONTROLLER_TICK_TIME_ms)
		);
	}
}

void handle_state(EventBits_t &current_state, Controller *controller) {
	switch (current_state) {
		case ControllerState_e::IDLE:
			break;
		case ControllerState_e::CONTROL:
			control_loop(controller);
			break;
		case ControllerState_e::WINDUP:
			break;
		case ControllerState_e::WINDOWN:
			break;
		default:
			break;
	}
}

void handle_error(EventBits_t &current_state, Controller *controller) {
	EventBits_t error_state = current_state & ~(ControllerState_e::ERROR);

	// Handle error
}

void control_loop(Controller *controller) {
	float control_signal = 0.0f;
	controller->loop();

	control_signal = controller->get_control_point().voltage;
}
