/**
 * @file pid_controller.hpp
 * @author ACMAX (aavaloscorrales@gmail.com)
 * @brief
 * @version 0.1
 * @date 2025-10-12
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once

#include "../controller.hpp"

#include <functional>

#include "num_calculus.hpp"

class PID : public Controller {
public:
	using ErrorFunction_t = std::function<float (float setpoint)>;
	PID (ErrorFunction_t error_function, float Kp, float Ki, float Kd);

	void                    setup()                override;
	Controller::ErrorType_t loop (float setpoint)  override;

	inline void set_integrator_saturators(float limit) {
		_integrator.saturatorMax() =  std::abs(limit);
		_integrator.saturatorMin() = -std::abs(limit);
	}

private:
	Integrator _integrator;
	Derivator  _derivator;
	ErrorFunction_t _error_function;

	float _kp;
	float _ki;
	float _kd;
};
