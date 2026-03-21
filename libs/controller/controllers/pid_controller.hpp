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
	PID (std::function<float ()> error_function, float Kp, float Ki, float Kd);

	void setup() override;
	void loop()  override;

private:
	Integrator _integrator;
	Derivator  _derivator;
	std::function<float ()> _error_function;

	float _kp;
	float _ki;
	float _kd;
};
