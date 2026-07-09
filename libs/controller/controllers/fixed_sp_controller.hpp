/**
 * @file fixed_sp_controller.hpp
 * @author ACMAX (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2026-06-28
 *
 * @copyright Copyright (c) 2026
 *
 */
#pragma once

#include <cmath>
#include <algorithm>

#include "controller.hpp"

#include "globals.hpp"
#include "esp_timer.h"

#include "dc_plant.hpp"
#include "telemetry_task.hpp"
#include "num_calculus.hpp"


struct BzSetpoint_t {
	float start_time = 0.0f;
	float trans_time = 0.0f;
	float setpoint   = 0.0f;
};

template <int n_setpoints>
class FixedSPController : public Controller {
public:
	FixedSPController(
		BzSetpoint_t *setpoints, float *r_values,
		float K1, float K2, float Ki, float Nu
	);

	void setup() override;
	void loop()  override;
protected:
private:
	static int constexpr       N_R = 6;
	BzSetpoint_t               _setpoints[n_setpoints];
	float                      _K1           =  0.0f;
	float                      _K2           =  0.0f;
	float                      _Ki           =  0.0f;
	float                      _Nu           =  0.0f;
	float                      _r[N_R]       = {0.0f};
	float                      _start_time_s =  0.0f;
	float                      _z            =  0.0f;
	int                        _current_setpoint = 0;
	uint64_t                   _start_time_us= 0L;

	inline float get_x(float time) {
		return (time - _setpoints[_current_setpoint].start_time)
		       /_setpoints[_current_setpoint].trans_time;
	}

	inline float get_bezier(float time) {
		float bezier = 0.0f;
		for (int i=0; i<N_R; i++) {
			bezier += _r[i]
			         *std::pow(get_x(time), (float)(i+5));
		}
		return bezier;
	}
};

inline bool operator< (
	BzSetpoint_t const &a,
	BzSetpoint_t const &b
) {
	return a.start_time < b.start_time;
}


template <int n_setpoints>
FixedSPController<n_setpoints>::FixedSPController(
	BzSetpoint_t *setpoints,
	float *r_values,
	float K1,
	float K2,
	float Ki,
	float Nu
) : Controller(),
	_K1(K1),
	_K2(K2),
	_Ki(Ki),
	_Nu(Nu),
	_estimator(SAMPLE_PARAMS, MODEL_SIM_TIME_s),
	_observer (SAMPLE_PARAMS, SAMPLE_OBS_PRMS, MODEL_SIM_TIME_s)
{
	if (!setpoints || !r_values) {
		return;
	}
	for (int i=0; i<n_setpoints; i++) {
		_setpoints[i] = setpoints[i];
		_r        [i] = r_values[i];
	}
	std::sort(_setpoints, _setpoints+N_R);
}
template <int n_setpoints>
void FixedSPController<n_setpoints>::setup() {
	_estimator.reset();
	_observer .reset();
	_start_time_us = esp_timer_get_time();
	_start_time_s  = _start_time_us*1e-6;
	_z = 0.0f;
	_current_setpoint = 0;
}
template <int n_setpoints>
void FixedSPController<n_setpoints>::loop() {
	static uint8_t n_ticks = 1;
	constexpr float    SIMULATED_LOAD   = 1.0f;
	float voltage_setpoint  = get_control_point().voltage;
	const float start_delta = esp_timer_get_time()*1e-6 - _start_time_s;
	float reference_w = 0.0f;
	const float speed_w = read_speed_rad_s();
	const float current = read_current();

	while (   _current_setpoint < (n_setpoints-1)
	    && start_delta > _setpoints[_current_setpoint+1].start_time
	) {
		_current_setpoint++;
	}

	reference_w = _setpoints[_current_setpoint].setpoint;
	if (
		start_delta < _setpoints[_current_setpoint].start_time
		             +_setpoints[_current_setpoint].trans_time
	) {
		float diff_setpoints = _setpoints[_current_setpoint].setpoint;
		if (_current_setpoint > 0) {
			diff_setpoints -= _setpoints[_current_setpoint-1].setpoint;
		}
		reference_w = diff_setpoints*get_bezier(start_delta);
	}

	interpolate_simulation (
		_start_time_us,
		_estimator,
		_observer,
		voltage_setpoint,
		SIMULATED_LOAD
	);

	voltage_setpoint = -_K1*speed_w
	                   -_K2*current
	                   -_Ki*_z
	                   +_Nu*reference_w;
	_z += _estimator.state().w_rad_s - reference_w;

	if (n_ticks >= 4) {
		task::telemetry::telemetry_data_t package = {
			.timestamp      = start_delta,
			.setpoint       = reference_w,
			.set_voltage    = voltage_setpoint,
			.w_rad_s        = speed_w,
			.I_amp          = current,
			.estimated_load = estimated_load_nm()
		};
		xQueueSend(
			task::telemetry::TelemetryTask::get_instance().data_queue(),
			&package,
			0
		);
		//(void)printf( "%10.3e"
		//             ",%10.3e"
		//             ",%10.3e"
		//             ",%10.3e"
		//             ",%10.3e"
		//             ",%10.3e"
		//             ",%10.3e\n",
		//	//_start_time_s,
		//	start_delta,
		//	reference_w,
		//	_estimator.state().w_rad_s,
		//	_estimator.state().I_amp,
		//	_observer .state().w_rad_s,
		//	_observer .state().I_amp,
		//	_observer .estimated_load()
		//);
		n_ticks = 0;
	}
	n_ticks++;

	set_voltage(voltage_setpoint);
}
