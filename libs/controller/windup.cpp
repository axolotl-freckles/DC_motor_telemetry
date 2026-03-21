/**
 * @file windup.cpp
 * @author ACMAX (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2025-08-13
 *
 * @copyright Copyright (c) 2025
 *
 */
#include "windup.hpp"

Windup::Windup()             : _period(0.0)    {}
Windup::Windup(float period) : _period(period) {}

ControlPoint LinearWindup::step(float delta_time) const {
	float proportion = delta_time / _period;

	return (ControlPoint) {
		.voltage = _st_voltage + proportion*_volt_slope
	};
}

void LinearWindup::set_st_voltage(float voltage) {
	_st_voltage = voltage;
	_volt_slope = _en_voltage - _st_voltage;
}
void LinearWindup::set_en_voltage(float voltage) {
	_en_voltage = voltage;
	_volt_slope = _en_voltage - _st_voltage;
}

LinearWindup::LinearWindup(
	float period,
	float start_voltage,
	float end_voltage
) : Windup(period)
	, _st_voltage(start_voltage)
	, _en_voltage(end_voltage)
	, _volt_slope(_en_voltage - _st_voltage)
{ }