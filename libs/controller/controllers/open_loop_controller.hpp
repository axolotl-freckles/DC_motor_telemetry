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

#include "freertos/FreeRTOS.h"

#include "dc_plant.hpp"

class OpenLoop : public Controller {
public:
	OpenLoop (QueueHandle_t voltage_q);

	void setup() override;
	void loop()  override;
private:
	QueueHandle_t              _voltage_q;

	DCPlant::EulerDCMotorModel _estimator;
	DCPlant::DCMotorObserver   _observer;
};

// TEMPORARY CONSTANTS; TODO: remove
constexpr double SAMPLE_TIME_s = 0.001;

const DCPlant::dc_parameters params = {
	.res_ohm      = 6.6,
	.inductance   = 0.00815,
	.moment_kg_m2 = 0.004,
	.viscous_u    = 0.00132,
	.Kt_Nm_A      = 0.436,
	.Kb_V_rad_s   = 0.436
};
const DCPlant::DCMotorObserver::EstimationParams es_prms = {
	.alfa_1 = 0.00001,
	.alfa_2 = 0.00001,
	.alfa_3 = 1.00000000001,
	.k_1     = 0.000000001,
	.k_2     = 0.00000001,
	.k_3     = 0.100000001,
};
