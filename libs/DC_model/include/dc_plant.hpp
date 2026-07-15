/**
 * @file dc_plant.hpp
 * @author ACMAX (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2026-06-03
 *
 * @copyright Copyright (c) 2026
 *
 */
#pragma once

#include <cstdint>

namespace DCPlant {

struct dc_parameters
{
	float res_ohm      = 0.0f; /* Amature resistance          */
	float inductance   = 0.0f; /* Inductance of amature       */
	float moment_kg_m2 = 0.0f; /* Moment of inertia           */
	float viscous_u    = 0.0f; /* Viscous friction coeficient */
	float Kt_Nm_A      = 0.0f; /* Torque constant             */
	float Kb_V_rad_s   = 0.0f; /* Back-emf constant           */
};

struct dc_state {
	float w_rad_s = 0.0f; /* Angular speed   */
	float I_amp   = 0.0f; /* Amature current */
};

class EulerDCMotorModel {
public:
	EulerDCMotorModel(EulerDCMotorModel   &other);
	EulerDCMotorModel(const dc_parameters &parameters,
	                  float                sample_time_s
	);

	/**
	 * @brief Calculate the next iteration of the simulation
	 *
	 * @param amature_volt Voltage applied to the amature
	 * @param load_Nm      Load applied to the motor
	 * @return Angular speed of the engine in rad/s
	 */
	float step (float const amature_volt, float const load_Nm);

	void reset();

	const dc_parameters& parameters () const { return _parameters;    }
	float                sample_time() const { return _sample_time_s; }
	const dc_state     & state      () const { return _state;         }
protected:
private:
	dc_parameters _parameters;
	float         _sample_time_s;

	dc_state      _state;
};

class DCMotorObserver {
public:
	struct EstimationParams {
		float alfa_1 = 0;
		float alfa_2 = 0;
		float alfa_3 = 0;

		float k_1 = 0;
		float k_2 = 0;
		float k_3 = 0;
	};

	struct EstimationResults {
		float w_rad_s = 0;
		float load_Nm = 0;
	};

	DCMotorObserver(DCMotorObserver   &other);
	DCMotorObserver(EulerDCMotorModel &other, EstimationParams &es_params);
	DCMotorObserver(const dc_parameters    &parameters,
	                const EstimationParams &es_params,
	                float                   sample_time_s
	);

	EstimationResults step (
		const float    amature_volt,
		const dc_state correct_state
	);

	void reset();

	const dc_parameters& parameters    () const { return _parameters;     }
	float                sample_time   () const { return _sample_time_s;  }
	const dc_state     & state         () const { return _state;          }
	float                estimated_load() const { return _estimated_load; }
protected:
private:
	dc_parameters    _parameters;
	EstimationParams _es_params;
	float            _sample_time_s;

	dc_state         _state;
	float            _estimated_load;
};

class DCMotorObserver_64 {
public:
	struct EstimationResults {
		int64_t I_amp_sh   = 0;
		int64_t load_Nm_sh = 0;
	};

	inline static int64_t to_repr  (float   value) { return (int64_t)(value*ONE_SH); }
	inline static float   from_repr(int64_t value) { return           value/ONE_SHf; }
	constexpr static int64_t to_repr_ctxpr  (float   value) { return (int64_t)(value*ONE_SH); }
	constexpr static float   from_repr_ctxpr(int64_t value) { return           value/ONE_SHf; }

	EstimationResults step(
		const int64_t amature_volt_sh,
		const int64_t w_rad_s_sh
	);

	void reset();

	float                 sample_time     () const;
	const   dc_state      state           () const;
	float                 estimated_load  () const;
	int64_t               estimated_load_i() const {return _estimated_load;}

	DCMotorObserver_64(
		const dc_parameters                     &parameters,
		const DCMotorObserver::EstimationParams &es_params,
		float                                    sample_time_s
	);

	inline static int64_t mul_fixed(const int64_t a, const int64_t b) {
		const int64_t e_a = a/ONE_SH;
		const int64_t e_b = b/ONE_SH;
		const int64_t f_a = a - e_a*ONE_SH;
		const int64_t f_b = b - e_b*ONE_SH;

		return e_a*e_b*ONE_SH + e_a*f_b + e_b*f_a + f_a*f_b/ONE_SH;
	}
private:
	static constexpr int64_t ONE_SH  = (1LL<<33) - 1LL;
	static constexpr float   ONE_SHf = (float)ONE_SH;
	static constexpr int64_t FILTER_RC = (int64_t)(0.80f*ONE_SH);
	/* ----------------------------- */
	const int64_t _sample_time_s  = 0;

	const int64_t _Tp_1 = 0;
	const int64_t _Tp_2 = 0;

	const int64_t _Wp_1 = 0;
	const int64_t _Wp_2 = 0;
	const int64_t _Wp_3 = 0;
	const int64_t _Wp_4 = 0;
	const int64_t _Wp_5 = 0;

	const int64_t _Ip_1 = 0;
	const int64_t _Ip_2 = 0;
	const int64_t _Ip_3 = 0;
	const int64_t _Ip_4 = 0;
	const int64_t _Ip_5 = 0;

	/* dc_state         _state;      */
	int64_t _w_rad_s_filtered = 0;
	int64_t _w_rad_s          = 0; /* Angular speed   */
	int64_t _I_amp            = 0; /* Amature current */
	/* ----------------------------- */
	int64_t _estimated_load = 0;
};

}

const DCPlant::dc_parameters SAMPLE_PARAMS = {
	.res_ohm      = 6.6f,
	.inductance   = 0.00815f,
	.moment_kg_m2 = 0.004f,
	.viscous_u    = 0.00132f,
	.Kt_Nm_A      = 0.436f,
	.Kb_V_rad_s   = 0.436f
};
const DCPlant::DCMotorObserver::EstimationParams SAMPLE_OBS_PRMS = {
	.alfa_1 = -0.00001f,
	.alfa_2 = -0.00001f,
	.alfa_3 = -1.00000000001f,
	.k_1    =  0.000000001f,
	.k_2    =  0.00000001f,
	.k_3    =  0.100000001f,
};
