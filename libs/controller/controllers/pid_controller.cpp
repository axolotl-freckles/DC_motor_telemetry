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

PID::PID (std::function<float ()> error_function, float Kp, float Ki, float Kd)
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
	set_voltage(1.0);
}
void PID::loop() {
	float error = _error_function();
	float u = _kp*error + _kd*_derivator(error) + _ki*_integrator(error);
	set_voltage(u);
}

