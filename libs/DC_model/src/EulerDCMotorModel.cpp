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
	float                sample_time_s
) :
	_parameters(parameters),
	_sample_time_s(sample_time_s)
{ }

float DCPlant::EulerDCMotorModel::step (
	float const amature_volt,
	float const load_Nm
) {
	const float y = _state.w_rad_s;
	DCPlant::dc_state new_state;

	const float moment_per_sample = _sample_time_s/_parameters.moment_kg_m2;

	const float W_grp_A = -_state.w_rad_s*_parameters.viscous_u*moment_per_sample;
	const float W_grp_B =  _state.I_amp*_parameters.Kt_Nm_A*moment_per_sample;
	const float W_grp_C = -load_Nm*moment_per_sample;

	const float inductance_per_sample = _sample_time_s/_parameters.inductance;

	const float I_grp_A = -_state.w_rad_s*_parameters.Kb_V_rad_s*inductance_per_sample;
	const float I_grp_B = -_state.I_amp*_parameters.res_ohm*inductance_per_sample;
	const float I_grp_C =  inductance_per_sample*amature_volt;

	new_state.w_rad_s = _state.w_rad_s + W_grp_A + W_grp_B + W_grp_C;
	new_state.I_amp   = _state.I_amp   + I_grp_A + I_grp_B + I_grp_C;

	_state = new_state;
	return y;
}

void DCPlant::EulerDCMotorModel::reset() {
	_state = {
		.w_rad_s = 0.0f,
		.I_amp   = 0.0f
	};
}
