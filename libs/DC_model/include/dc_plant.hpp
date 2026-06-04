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
	double res_ohm      = 0.0f; /* Amature resistance          */
	double inductance   = 0.0f; /* Inductance of amature       */
	double moment_kg_m2 = 0.0f; /* Moment of inertia           */
	double viscous_u    = 0.0f; /* Viscous friction coeficient */
	double Kt = 0.0f; /* ??? */
	double Kb = 0.0f; /* ??? */
};

struct dc_state {
	double w_rad_s = 0.0f; /* Angular speed   */
	double I_amp   = 0.0f; /* Amature current */
};

class EulerDCMotorModel {
public:
	EulerDCMotorModel(EulerDCMotorModel &other);
	EulerDCMotorModel(dc_parameters &parameters, double sample_time_s);

	double step(double const amature_volt, double const load_Nm);
protected:
private:
	dc_parameters _parameters;
	double        _sample_time_s;

	dc_state      _state;
};

}
