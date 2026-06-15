/**
 * @file DCMotorObserver.cpp
 * @author ACMAX (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2026-06-08
 *
 * @copyright Copyright (c) 2026
 *
 */
#include "dc_plant.hpp"

//#define sign(x) ((x<0)? -1 : 1)

template <typename _T>
static int sign(const _T value) { return (value<0)? -1 : 1; }

DCPlant::DCMotorObserver::DCMotorObserver(
	EulerDCMotorModel &other,
	EstimationParams  &es_params
) :
	_parameters      (other.parameters ()),
	_es_params       (es_params),
	_sample_time_s   (other.sample_time()),
	_estimated_load(0.0)
{ }
DCPlant::DCMotorObserver::DCMotorObserver(
	const dc_parameters    &parameters,
	const EstimationParams &es_params,
	double                  sample_time_s
) :
	_parameters      (parameters),
	_es_params       (es_params),
	_sample_time_s   (sample_time_s),
	_estimated_load(0.0)
{ }

DCPlant::DCMotorObserver::EstimationResults DCPlant::DCMotorObserver::step (
	const double            amature_volt,
	const DCPlant::dc_state correct_state
) {
	const double y = _state.w_rad_s;
	DCPlant::dc_state new_state;
	DCPlant::dc_state error     = {
		.w_rad_s = correct_state.w_rad_s - _state.w_rad_s,
		.I_amp   = correct_state.I_amp   - _state.I_amp
	};

	const double new_estimated_load = _estimated_load
	+ _sample_time_s*( 0.0
		+ _es_params.alfa_3*error.w_rad_s
		+ _es_params.k_3*sign(error.w_rad_s)
	);

	new_state.w_rad_s = _state.w_rad_s;
	new_state.I_amp   = _state.I_amp;

	new_state.w_rad_s += _sample_time_s*(
		- _state.w_rad_s*_parameters.viscous_u/_parameters.moment_kg_m2
		+ _state.I_amp  *_parameters.Kt_Nm_A  /_parameters.moment_kg_m2
		- _estimated_load                     /_parameters.moment_kg_m2
		+ _es_params.alfa_1*error.w_rad_s
		+ _es_params.k_1   *sign(error.w_rad_s)
	);

	new_state.I_amp   += _sample_time_s*(
		- _state.w_rad_s*_parameters.Kb_V_rad_s/_parameters.inductance
		- _state.I_amp  *_parameters.res_ohm   /_parameters.inductance
		+ amature_volt                         /_parameters.inductance
		+ _es_params.alfa_2*error.w_rad_s
		+ _es_params.k_2   *sign(error.w_rad_s)
	);

	_state          = new_state;
	_estimated_load = new_estimated_load;
	return DCPlant::DCMotorObserver::EstimationResults{
		.w_rad_s = y,
		.load_Nm = new_estimated_load
	};
}