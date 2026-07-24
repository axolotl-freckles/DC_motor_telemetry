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

using namespace DCPlant;

template <typename _T>
static int sign(const _T value) { return (value<0)? -1 : 1; }

DCPlant::DCMotorObserver::DCMotorObserver(
	EulerDCMotorModel &other,
	EstimationParams  &es_params
) :
	_parameters      (other.parameters ()),
	_es_params       (es_params),
	_sample_time_s   (other.sample_time()),
	_estimated_load(0.0f)
{ }
DCPlant::DCMotorObserver::DCMotorObserver(
	const dc_parameters    &parameters,
	const EstimationParams &es_params,
	float                   sample_time_s
) :
	_parameters      (parameters),
	_es_params       (es_params),
	_sample_time_s   (sample_time_s),
	_estimated_load(0.0f)
{ }

DCPlant::DCMotorObserver::EstimationResults DCPlant::DCMotorObserver::step (
	const float             amature_volt,
	const DCPlant::dc_state correct_state
) {
	const float y = _state.w_rad_s;
	DCPlant::dc_state new_state;
	DCPlant::dc_state error     = {
		.w_rad_s = correct_state.w_rad_s - _state.w_rad_s,
		.I_amp   = correct_state.I_amp   - _state.I_amp
	};

	const float new_estimated_load = _estimated_load
	+ _sample_time_s*(
		+ _es_params.alfa_3*error.w_rad_s
		+ _es_params.k_3   *sign(error.w_rad_s)
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
void DCPlant::DCMotorObserver::reset() {
	_estimated_load = 0.0f;
	_state = {
		.w_rad_s = 0.0f,
		.I_amp   = 0.0f
	};
}

/*###################### INT VERSION #########################################*/
void DCPlant::DCMotorObserver_64::reset() {
	_w_rad_s        = 0;
	_I_amp          = 0;
	_estimated_load = 0;
}

DCPlant::DCMotorObserver_64::EstimationResults DCPlant::DCMotorObserver_64::step (
	const int64_t amature_volt_sh,
	const int64_t w_rad_s_sh
) {
	// _w_rad_s_filtered = mul_fixed(w_rad_s_sh, FILTER_RC)
	//                    +mul_fixed(ONE_SH - FILTER_RC, _w_rad_s_filtered);
	const int64_t er_w = w_rad_s_sh - _w_rad_s;

	const int64_t new_estimated_load =
		  _estimated_load
		+ mul_fixed(_Tp_1, er_w)
		+ _Tp_2*sign(er_w)
	;

	const int64_t new_w_rad_s =
		_w_rad_s
		+ mul_fixed(_Wp_1, _w_rad_s)
		+ mul_fixed(_Wp_2, _I_amp)
		+ mul_fixed(_Wp_3, _estimated_load)

		+ mul_fixed(_Wp_4, er_w)
		+ _Wp_5*sign(er_w)
	;
	const int64_t new_I_amp  =
		_I_amp
		+ mul_fixed(_Ip_1, _w_rad_s)
		+ mul_fixed(_Ip_2, _I_amp)
		+ mul_fixed(_Ip_3, amature_volt_sh)
	
		+ mul_fixed(_Ip_4, er_w)
		+ _Ip_5*sign(er_w)
	;

	_w_rad_s = new_w_rad_s;
	_I_amp   = new_I_amp;
	_estimated_load = new_estimated_load;
	return DCPlant::DCMotorObserver_64::EstimationResults{
		.I_amp_sh   = new_I_amp,
		.load_Nm_sh = new_estimated_load
	};
}

float DCPlant::DCMotorObserver_64::sample_time   () const {
	return DCPlant::DCMotorObserver_64::from_repr(_sample_time_s);
}
const dc_state      DCPlant::DCMotorObserver_64::state          () const {
	return (dc_state) {
		.w_rad_s = DCPlant::DCMotorObserver_64::from_repr(_w_rad_s),
		.I_amp   = DCPlant::DCMotorObserver_64::from_repr(_I_amp),
	};
}
float DCPlant::DCMotorObserver_64::estimated_load () const {
	return DCPlant::DCMotorObserver_64::from_repr(_estimated_load);
}

DCPlant::DCMotorObserver_64::DCMotorObserver_64(
	const dc_parameters                     &parameters,
	const DCMotorObserver::EstimationParams &es_params,
	float                                    sample_time_s
) :
	_sample_time_s(DCMotorObserver_64::to_repr(sample_time_s)),
	/* ----------------------------- */
	_Tp_1(DCMotorObserver_64::to_repr( sample_time_s*es_params.alfa_3 )),
	_Tp_2(DCMotorObserver_64::to_repr( sample_time_s*es_params.k_3    )),

	_Wp_1(DCMotorObserver_64::to_repr(-sample_time_s*parameters.viscous_u/parameters.moment_kg_m2 )),
	_Wp_2(DCMotorObserver_64::to_repr( sample_time_s*parameters.Kt_Nm_A/parameters.moment_kg_m2 )),
	_Wp_3(DCMotorObserver_64::to_repr(-sample_time_s/parameters.moment_kg_m2 )),
	_Wp_4(DCMotorObserver_64::to_repr( sample_time_s*es_params.alfa_1 )),
	_Wp_5(DCMotorObserver_64::to_repr( sample_time_s*es_params.k_1 )),

	_Ip_1(DCMotorObserver_64::to_repr(-sample_time_s*parameters.Kb_V_rad_s/parameters.inductance )),
	_Ip_2(DCMotorObserver_64::to_repr(-sample_time_s*parameters.res_ohm/parameters.inductance )),
	_Ip_3(DCMotorObserver_64::to_repr( sample_time_s/parameters.inductance )),
	_Ip_4(DCMotorObserver_64::to_repr( sample_time_s*es_params.alfa_2 )),
	_Ip_5(DCMotorObserver_64::to_repr( sample_time_s*es_params.k_2 ))
{ }