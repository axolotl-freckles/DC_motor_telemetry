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
	float   _sample_time_s;

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
		const float amature_volt,
		const dc_state    correct_state
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
	float      _sample_time_s;

	dc_state         _state;
	float      _estimated_load;
};

}
