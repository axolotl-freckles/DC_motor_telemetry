/**
 * @file windown.hpp
 * @author ACMAX (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2025-10-29
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once

#include "controller_types.hpp"

using control::ControlPoint;

class Winddown {
public:
	virtual float step(float delta_time_s) const = 0;

	inline float period()          const { return _period; }
	inline bool  assert_winddown() const {
		return this->step(0.0f) > this->step(_period);
	}
	virtual void set_st_setpoint(float voltage) = 0;
	virtual void set_en_setpoint(float voltage) = 0;

	virtual float get_en_setpoint() const = 0;

	Winddown();
	Winddown(float period);
	virtual ~Winddown() { }
protected:
	float _period;
};
