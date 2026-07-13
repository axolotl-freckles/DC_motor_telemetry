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
		.w_rad_s = _state.w_rad_s - correct_state.w_rad_s,
		.I_amp   = _state.I_amp   - correct_state.I_amp
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
	const int64_t er_w = _w_rad_s - w_rad_s_sh;
	int64_t new_w_rad_s = 0;
	int64_t new_I_amp   = 0;

	const int64_t new_estimated_load = _estimated_load
	+ _sample_time_s*(
		+ _alfa_3*er_w
		+ _k_3   *sign(er_w)
	);

	new_w_rad_s = _w_rad_s;
	new_I_amp   = _I_amp;

	new_w_rad_s += _sample_time_s*(
		- _w_rad_s*_viscous_u/_moment_kg_m2
		+ _I_amp  *_Kt_Nm_A  /_moment_kg_m2
		- _estimated_load    /_moment_kg_m2
		+ _alfa_1*er_w
		+ _k_1   *sign(er_w)
	);

	new_I_amp   += _sample_time_s*(
		- _w_rad_s*_Kb_V_rad_s/_inductance
		- _I_amp  *_res_ohm   /_inductance
		+ amature_volt_sh     /_inductance
		+ _alfa_2*er_w
		+ _k_2   *sign(er_w)
	);

	_w_rad_s = new_w_rad_s;
	_I_amp   = new_I_amp;
	_estimated_load = new_estimated_load;
	return DCPlant::DCMotorObserver_64::EstimationResults{
		.I_amp_sh   = new_I_amp,
		.load_Nm_sh = new_estimated_load
	};
}

const dc_parameters DCPlant::DCMotorObserver_64::parameters     () const {
	return (dc_parameters) {
		.res_ohm        = DCPlant::DCMotorObserver_64::from_repr(_res_ohm     ),
		.inductance     = DCPlant::DCMotorObserver_64::from_repr(_inductance  ),
		.moment_kg_m2   = DCPlant::DCMotorObserver_64::from_repr(_moment_kg_m2),
		.viscous_u      = DCPlant::DCMotorObserver_64::from_repr(_viscous_u   ),
		.Kt_Nm_A        = DCPlant::DCMotorObserver_64::from_repr(_Kt_Nm_A     ),
		.Kb_V_rad_s     = DCPlant::DCMotorObserver_64::from_repr(_Kb_V_rad_s  ),
	};
}
float                DCPlant::DCMotorObserver_64::sample_time   () const {
	return DCPlant::DCMotorObserver_64::from_repr(_sample_time_s);
}
const dc_state      DCPlant::DCMotorObserver_64::state          () const {
	return (dc_state) {
		.w_rad_s = DCPlant::DCMotorObserver_64::from_repr(_w_rad_s),
		.I_amp   = DCPlant::DCMotorObserver_64::from_repr(_I_amp),
	};
}
float               DCPlant::DCMotorObserver_64::estimated_load () const {
	return DCPlant::DCMotorObserver_64::from_repr(_estimated_load);
}

DCPlant::DCMotorObserver_64::DCMotorObserver_64(
	const dc_parameters                     &parameters,
	const DCMotorObserver::EstimationParams &es_params,
	float                                    sample_time_s
) :
	/* dc_parameters    _parameters; */
	_res_ohm      (DCMotorObserver_64::to_repr(parameters.res_ohm     )),
	_inductance   (DCMotorObserver_64::to_repr(parameters.inductance  )),
	_moment_kg_m2 (DCMotorObserver_64::to_repr(parameters.moment_kg_m2)),
	_viscous_u    (DCMotorObserver_64::to_repr(parameters.viscous_u   )),
	_Kt_Nm_A      (DCMotorObserver_64::to_repr(parameters.Kt_Nm_A     )),
	_Kb_V_rad_s   (DCMotorObserver_64::to_repr(parameters.Kb_V_rad_s  )),
	/* EstimationParams _es_params;  */
	_alfa_1       (DCMotorObserver_64::to_repr(es_params.alfa_1)),
	_alfa_2       (DCMotorObserver_64::to_repr(es_params.alfa_2)),
	_alfa_3       (DCMotorObserver_64::to_repr(es_params.alfa_3)),
	_k_1          (DCMotorObserver_64::to_repr(es_params.k_1   )),
	_k_2          (DCMotorObserver_64::to_repr(es_params.k_2   )),
	_k_3          (DCMotorObserver_64::to_repr(es_params.k_3   )),
	/* ----------------------------- */
	_sample_time_s(DCMotorObserver_64::to_repr(sample_time_s))
{ }