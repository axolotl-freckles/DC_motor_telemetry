/**
 * @file windup.hpp
 * @author ACMAX (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2025-08-13
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once

#include "controller_types.hpp"

class Windup {
public:
	virtual float step(float delta_time_s) const = 0;

	inline float period()        const { return _period; }
	inline bool  assert_windup() const {
		return this->step(0.0f) < this->step(_period);
	}
	virtual void set_st_setpoint(float setpoint) = 0;
	virtual void set_en_setpoint(float setpoint) = 0;

	virtual float get_en_setpoint() const = 0;

	Windup();
	Windup(float period);
	virtual ~Windup() { }
protected:
	float _period;
};
