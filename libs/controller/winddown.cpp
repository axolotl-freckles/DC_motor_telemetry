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

Winddown::Winddown()             : _period(0.0)    {}
Winddown::Winddown(float period) : _period(period) {}

ControlPoint LinearWinddown::step(float delta_time) const {
	float proportion = delta_time / _period;

	return (ControlPoint) {
		.amplitude = _st_amplitude + proportion*_amp_slope,
		.flux_speed {
			.value = _st_flx_speed + proportion*_flux_val_slope,
			.type = _speed_type
		}
	};
}

void LinearWinddown::set_st_amplitude(float amplitude) {
	_st_amplitude = amplitude;
	_amp_slope = _en_amplitude - _st_amplitude;
}
void LinearWinddown::set_en_amplitude(float amplitude) {
	_en_amplitude = amplitude;
	_amp_slope = _en_amplitude - _st_amplitude;
}
void LinearWinddown::set_st_flux_speed(float flux_speed_val) {
	_st_flx_speed = flux_speed_val;
	_flux_val_slope = _en_flx_speed - _st_flx_speed;
}
void LinearWinddown::set_en_flux_speed(float flux_speed_val) {
	_en_flx_speed = flux_speed_val;
	_flux_val_slope = _en_flx_speed - _st_flx_speed;
}

LinearWinddown::LinearWinddown(
	float period,
	float start_amplitude, float start_flxSpeed_val,
	float end_amplitude,   float end_flxSpeed_val,
	FluxSpeed_t flux_speed_type
) : Winddown(period)
	, _st_amplitude(start_amplitude)
	, _en_amplitude(end_amplitude)
	, _amp_slope(_en_amplitude - _st_amplitude)
	, _st_flx_speed(start_flxSpeed_val)
	, _en_flx_speed(end_flxSpeed_val)
	, _flux_val_slope(_en_flx_speed - _st_flx_speed)
	, _speed_type(flux_speed_type)
{ }