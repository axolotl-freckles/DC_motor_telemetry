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
	double res_ohm      = 0.0; /* Amature resistance          */
	double inductance   = 0.0; /* Inductance of amature       */
	double moment_kg_m2 = 0.0; /* Moment of inertia           */
	double viscous_u    = 0.0; /* Viscous friction coeficient */
	double Kt_Nm_A      = 0.0; /* Torque constant             */
	double Kb_V_rad_s   = 0.0; /* Back-emf constant           */
};

struct dc_state {
	double w_rad_s = 0.0; /* Angular speed   */
	double I_amp   = 0.0; /* Amature current */
};

class EulerDCMotorModel {
public:
	EulerDCMotorModel(EulerDCMotorModel &other);
	EulerDCMotorModel(dc_parameters &parameters, double sample_time_s);

	/**
	 * @brief Calculate the next iteration of the simulation
	 *
	 * @param amature_volt Voltage applied to the amature
	 * @param load_Nm      Load applied to the motor
	 * @return Angular speed of the engine in rad/s
	 */
	double step(double const amature_volt, double const load_Nm);
protected:
private:
	dc_parameters _parameters;
	double        _sample_time_s;

	dc_state      _state;
};

}
