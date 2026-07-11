/**
 * @file LinearWinddown.cpp
 * @author ACMAX (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2026-07-10
 *
 * @copyright Copyright (c) 2026
 *
 */
#include "LinearWinddown.hpp"

float LinearWinddown::step(float delta_time) const {
	float proportion = delta_time / _period;

	return _st_setpoint + proportion*_setpoint_slope;
}

void LinearWinddown::set_st_setpoint(float setpoint) {
	_st_setpoint = setpoint;
	_setpoint_slope = _en_setpoint - _st_setpoint;
}
void LinearWinddown::set_en_setpoint(float setpoint) {
	_en_setpoint = setpoint;
	_setpoint_slope = _en_setpoint - _st_setpoint;
}

LinearWinddown::LinearWinddown(
	float period,
	float start_setpoint,
	float end_setpoint
) : Winddown(period)
	, _st_setpoint(start_setpoint)
	, _en_setpoint(end_setpoint)
	, _setpoint_slope(_en_setpoint - _st_setpoint)
{ }
