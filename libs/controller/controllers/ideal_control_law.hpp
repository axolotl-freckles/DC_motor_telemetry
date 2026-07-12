/**
 * @file ideal_control_law.hpp
 * @author ACMAX (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2026-07-11
 *
 * @copyright Copyright (c) 2026
 *
 */
#pragma once

#include "controller.hpp"

class IdealControlLaw : public Controller {
public:
	void        setup()               override;
	ErrorType_t loop (float setpoint) override;

	IdealControlLaw(float K1, float K2, float Ki, float Nu);
protected:
private:
	float _K1 = 0.0f;
	float _K2 = 0.0f;
	float _Ki = 0.0f;
	float _Nu = 0.0f;
	float _z  = 0.0f;

	IdealControlLaw();
};
