/**
 * @file num_calculus.cpp
 * @author ACMAX (aavaloscorales@gmail.com)
 * @brief
 * @version 0.1
 * @date 2025-03-28
 *
 * @copyright Copyright (c) 2025
 *
 */
#include "num_calculus.hpp"

Integrator::Integrator (
	const float SAMPLE_TIME_s,
	const float starting_value,
	const float saturator_max,
	const float saturator_min
) :
	_SAMPLE_TIME_s(SAMPLE_TIME_s),
	_saturator_max(saturator_max),
	_saturator_min(saturator_min),
	_integral_acum(starting_value)
{}
Integrator::Integrator (const Integrator &_other)
:
	_SAMPLE_TIME_s(_other.sampleTime()),
	_saturator_max(_other.saturatorMax()),
	_saturator_min(_other.saturatorMin()),
	_integral_acum(_other.integralAcumulator())
{}

float Integrator::operator() (float value) {
	_integral_acum += value*_SAMPLE_TIME_s;
	return _integral_acum;
}


Derivator::Derivator (const float SAMPLE_TIME_s, const float starting_value)
:
	_SAMPLE_TIME_s(SAMPLE_TIME_s),
	_prev_val(starting_value)
{}
Derivator::Derivator (const Derivator &_other)
:
	_SAMPLE_TIME_s(_other.sampleTime()),
	_prev_val(_other.previousValue())
{}

float Derivator::operator() (float value) {
	float out = (value - _prev_val)/_SAMPLE_TIME_s;
	_prev_val = value;
	return out;
}