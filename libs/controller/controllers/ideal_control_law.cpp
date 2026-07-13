/**
 * @file ideal_control_law.cpp
 * @author ACMAX (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2026-07-11
 *
 * @copyright Copyright (c) 2026
 *
 */
#include "ideal_control_law.hpp"

#include "esp_timer.h"

#include "telemetry_task.hpp"

void IdealControlLaw::setup() {
	_z = 0.0f;
}

Controller::ErrorType_t IdealControlLaw::loop(float setpoint) {
	const float speed_w = read_speed_rad_s();
	const float current = read_current();
	const float voltage_setpoint = -_K1*speed_w
	                               -_K2*current
	                               -_Ki*_z
	                               +_Nu*setpoint;
	_z += speed_w - setpoint;

	set_voltage(voltage_setpoint);
	return Controller::ErrorType_t::OK;
}

IdealControlLaw::IdealControlLaw(float K1, float K2, float Ki, float Nu)
: Controller()
	,_K1(K1)
	,_K2(K2)
	,_Ki(Ki)
	,_Nu(Nu)
	,_z(0.0f)
{ }