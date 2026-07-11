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

	void                    setup()                override;
	Controller::ErrorType_t loop (float setpoint)  override;

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
