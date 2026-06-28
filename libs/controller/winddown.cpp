/**
 * @file windown.cpp
 * @author ACMAX (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2025-10-29
 *
 * @copyright Copyright (c) 2025
 *
 */
#include "winddown.hpp"

Winddown::Winddown()             : _period(0.0f)    {}
Winddown::Winddown(float period) : _period(period) {}

ControlPoint LinearWinddown::step(float delta_time) const {
	float proportion = delta_time / _period;

	return (ControlPoint) {
		.voltage = _st_voltage + proportion*_volt_slope
	};
}

void LinearWinddown::set_st_voltage(float voltage) {
	_st_voltage = voltage;
	_volt_slope = _en_voltage - _st_voltage;
}
void LinearWinddown::set_en_voltage(float voltage) {
	_en_voltage = voltage;
	_volt_slope = _en_voltage - _st_voltage;
}

LinearWinddown::LinearWinddown(
	float period,
	float start_voltage,
	float end_voltage
) : Winddown(period)
	, _st_voltage(start_voltage)
	, _en_voltage(end_voltage)
	, _volt_slope(_en_voltage - _st_voltage)
{ }