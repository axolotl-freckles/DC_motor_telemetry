/**
 * @file NoWindup.cpp
 * @author ACMAX (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2026-07-10
 *
 * @copyright Copyright (c) 2026
 *
 */
#include "NoWindup.hpp"

using control::ControlPoint;

float NoWindup::step(float delta_time_s) const {
	return _en_setpoint;
}

NoWindup::NoWindup() : Windup(0.0f)
	,_en_setpoint(0.0f)
{ }
NoWindup::NoWindup(float end_setpoint) : Windup(0.0f)
	,_en_setpoint(end_setpoint)
{ }
