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

using control::FluxSpeed;
using control::FluxSpeed_t;
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
	void set_st_amplitude(float amplitude);
	void set_en_amplitude(float amplitude);
	void set_st_flux_speed(float flux_speed_val);
	void set_en_flux_speed(float flux_speed_val);
	// void set_flux_speed_type(FluxSpeed_t type);

	inline const float& get_st_amplitude() const { return _st_amplitude; }
	inline const float& get_en_amplitude() const { return _en_amplitude; }
	inline const FluxSpeed get_st_flux_speed() const {
		return (FluxSpeed) {
			.value = _st_flx_speed,
			.type  = _speed_type
		};
	}
	inline const FluxSpeed get_en_flux_speed() const {
		return (FluxSpeed) {
			.value = _en_flx_speed,
			.type  = _speed_type
		};
	}
	inline const FluxSpeed_t& get_flux_speed_type() const { return _speed_type; }

	LinearWindup(
		float period_s,
		float start_amplitude, float start_flxSpeed_val,
		float end_amplitude,   float end_flxSpeed_val,
		FluxSpeed_t flux_speed_type = FluxSpeed_t::FREQUENCY
	);
	virtual ~LinearWindup() { }
private:
	float _st_amplitude;
	float _en_amplitude;
	float _amp_slope;

	float _st_flx_speed;
	float _en_flx_speed;
	float _flux_val_slope;
	FluxSpeed_t _speed_type;
};
