/**
 * @file EulerDCMotorModel.cpp
 * @author ACMAX (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2026-06-04
 *
 * @copyright Copyright (c) 2026
 *
 */
#include "dc_plant.hpp"

DCPlant::EulerDCMotorModel::EulerDCMotorModel (
	const dc_parameters &parameters,
	double               sample_time_s
) :
	_parameters(parameters),
	_sample_time_s(sample_time_s)
{ }

double DCPlant::EulerDCMotorModel::step (
	double const amature_volt,
	double const load_Nm
) {
	const double y = _state.w_rad_s;
	DCPlant::dc_state new_state;

	new_state.w_rad_s = _state.w_rad_s;
	new_state.I_amp   = _state.I_amp;

	new_state.w_rad_s += _sample_time_s*(
		- _state.w_rad_s*_parameters.viscous_u
		+ _state.I_amp  *_parameters.Kt_Nm_A
		- load_Nm
	) / _parameters.moment_kg_m2;
	new_state.I_amp   += _sample_time_s*(
		- _state.w_rad_s*_parameters.Kb_V_rad_s
		- _state.I_amp  *_parameters.res_ohm
		+ amature_volt
	) / _parameters.inductance;

	_state = new_state;
	return y;
}
