/**
 * @file open_loop_controller.hpp
 * @author ACMAX (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2026-06-17
 *
 * @copyright Copyright (c) 2026
 *
 */
#pragma once

#include "../controller.hpp"

#include "globals.hpp"

#include "freertos/FreeRTOS.h"

#include "dc_plant.hpp"

class OpenLoop : public Controller {
public:
	OpenLoop (QueueHandle_t voltage_q);

	void setup() override;
	void loop()  override;

	DCPlant::EulerDCMotorModel       &model   ()       { return _estimator; }
	DCPlant::DCMotorObserver         &observer()       { return _observer;  }
	const DCPlant::EulerDCMotorModel &model   () const { return _estimator; }
	const DCPlant::DCMotorObserver   &observer() const { return _observer;  }
private:
	QueueHandle_t              _voltage_q;

	DCPlant::EulerDCMotorModel _estimator;
	DCPlant::DCMotorObserver   _observer;
	uint64_t                   _start_time_us    = 0L;
	float                      _voltage_setpoint = 0.0f;
};

const DCPlant::dc_parameters params = {
	.res_ohm      = 6.6f,
	.inductance   = 0.00815f,
	.moment_kg_m2 = 0.004f,
	.viscous_u    = 0.00132f,
	.Kt_Nm_A      = 0.436f,
	.Kb_V_rad_s   = 0.436f
};
const DCPlant::DCMotorObserver::EstimationParams es_prms = {
	.alfa_1 = 0.00001f,
	.alfa_2 = 0.00001f,
	.alfa_3 = 1.00000000001f,
	.k_1     = 0.000000001f,
	.k_2     = 0.00000001f,
	.k_3     = 0.100000001f,
};
