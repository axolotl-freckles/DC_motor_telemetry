/**
 * @file NoWinddown.cpp
 * @author ACMAX (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2026-07-10
 *
 * @copyright Copyright (c) 2026
 *
 */
#include "NoWinddown.hpp"

float NoWinddown::step(float delta_time_s) const {
	return _en_setpoint;
}

NoWinddown::NoWinddown() : Winddown(0.0f)
	,_en_setpoint(0.0f)
{ }
NoWinddown::NoWinddown(float end_setpoint) : Winddown(0.0f)
	,_en_setpoint(end_setpoint)
{ }
