/**
 * @file LinearWinddown.hpp
 * @author ACMAX (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2026-07-10
 *
 * @copyright Copyright (c) 2026
 *
 */
#pragma once

#include "winddown.hpp"

class LinearWinddown : public Winddown {
public:
	float step(float delta_time_s) const;

	inline void set_period(float period) { _period = period; }
	void set_st_setpoint(float setpoint) override;
	void set_en_setpoint(float setpoint) override;

	inline float get_st_setpoint() const          { return _st_setpoint; }
	inline float get_en_setpoint() const override { return _en_setpoint; }

	LinearWinddown(
		float period_s,
		float start_setpoint,
		float end_setpoint
	);
	virtual ~LinearWinddown() { }
private:
	float _st_setpoint;
	float _en_setpoint;
	float _setpoint_slope;
};
