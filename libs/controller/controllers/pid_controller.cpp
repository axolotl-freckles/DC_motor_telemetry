/**
 * @file pid_controller.cpp
 * @author ACMAX (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2025-10-12
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "pid_controller.hpp"

#include "esp_timer.h"

#include "telemetry_task.hpp"

PID::PID (PID::ErrorFunction_t error_function, float Kp, float Ki, float Kd)
: Controller()
	, _integrator(get_sample_time_s())
	, _derivator(get_sample_time_s())
	, _error_function(error_function)
	, _kp(Kp)
	, _ki(Ki)
	, _kd(Kd)
{}

void PID::setup() {
	_integrator.setIntegralAcumulator(0.0f);
	set_voltage(1.0f);
}
Controller::ErrorType_t PID::loop(float setpoint) {
	static uint8_t n_ticks = 1;
	float error = _error_function(setpoint);
	float u = _kp*error + _kd*_derivator(error) + _ki*_integrator(error);

	if (n_ticks >= 4) {
		task::telemetry::telemetry_data_t package = {
			.timestamp      = esp_timer_get_time()*1e-6f,
			.setpoint       = setpoint,
			.set_voltage    = u,
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

	set_voltage(u);
	return Controller::ErrorType_t::OK;
}

