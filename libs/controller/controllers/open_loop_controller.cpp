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

OpenLoop::OpenLoop (QueueHandle_t voltage_q) :
	_voltage_q (voltage_q),
	_estimator (params, SAMPLE_TIME_s),
	_observer  (params, es_prms, SAMPLE_TIME_s)
{}

void OpenLoop::setup () {
	_estimator.reset();
	_observer .reset();
}

void OpenLoop::loop () {
	constexpr double SIMULATED_LOAD = 1.0;
	double voltage_setpoint = 0.0;
	(void)xQueuePeek(_voltage_q, &voltage_setpoint, portMAX_DELAY);

	_observer .step(voltage_setpoint, _estimator.state());
	_estimator.step(voltage_setpoint, SIMULATED_LOAD);

	set_voltage(voltage_setpoint);
}