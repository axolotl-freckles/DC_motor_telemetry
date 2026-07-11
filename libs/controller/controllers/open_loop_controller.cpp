/**
 * @file open_loop_controller.cpp
 * @author ACMAX (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2026-06-22
 *
 * @copyright Copyright (c) 2026
 *
 */
#include "open_loop_controller.hpp"

#include <stdio.h>

#include "esp_timer.h"

#include "telemetry_task.hpp"
#include "num_calculus.hpp"

OpenLoop::OpenLoop () : Controller() {}

void OpenLoop::setup () {}

Controller::ErrorType_t OpenLoop::loop (float setpoint) {
	static    uint8_t  n_ticks        = 1;

	if (n_ticks >= 4) {
		task::telemetry::telemetry_data_t package = {
			.timestamp      = esp_timer_get_time()*1e-6f,
			.setpoint       = setpoint,
			.set_voltage    = setpoint,
			.w_rad_s        = read_speed_rad_s(),
			.I_amp          = read_current(),
			.estimated_load = estimated_load_nm()
		};
		xQueueSend(
			task::telemetry::TelemetryTask::get_instance().data_queue(),
			&package,
			0
		);
		n_ticks = 0;
	}
	n_ticks++;

	set_voltage(setpoint);
	return Controller::ErrorType_t::OK;
}