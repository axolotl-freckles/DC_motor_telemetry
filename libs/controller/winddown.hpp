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
	virtual ControlPoint step(float delta_time_s) const = 0;

	inline float period()          const { return _period; }
	inline bool  assert_winddown() const {
		return this->step(0.0f) > this->step(_period);
	}

	Winddown();
	Winddown(float period);
	virtual ~Winddown() { }
protected:
	float _period;
};

class LinearWinddown : public Winddown {
public:
	ControlPoint step(float delta_time_s) const;

	inline void set_period(float period) { _period = period; }
	void set_st_voltage(float voltage);
	void set_en_voltage(float voltage);

	inline const float& get_st_voltage() const { return _st_voltage; }
	inline const float& get_en_voltage() const { return _en_voltage; }

	LinearWinddown(
		float period_s,
		float start_amplitude,
		float end_amplitude
	);
	virtual ~LinearWinddown() { }
private:
	float _st_voltage;
	float _en_voltage;
	float _volt_slope;
};