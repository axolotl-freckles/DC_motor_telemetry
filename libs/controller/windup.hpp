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

using control::ControlPoint;

class Windup {
public:
	virtual ControlPoint step(float delta_time_s) const = 0;

	inline float period()        const { return _period; }
	inline bool  assert_windup() const {
		return this->step(0.0f) < this->step(_period);
	}

	Windup();
	Windup(float period);
	virtual ~Windup() { }
protected:
	float _period;
};

class LinearWindup : public Windup {
public:
	ControlPoint step(float delta_time_s) const;

	inline void set_period(float period) { _period = period; }
	void set_st_voltage(float voltage);
	void set_en_voltage(float voltage);

	inline const float& get_st_voltage() const { return _st_voltage; }
	inline const float& get_en_voltage() const { return _en_voltage; }

	LinearWindup(
		float period_s,
		float start_amplitude,
		float end_amplitude
	);
	virtual ~LinearWindup() { }
private:
	float _st_voltage;
	float _en_voltage;
	float _volt_slope;
};
