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
	OpenLoop ();

	void                    setup()                override;
	Controller::ErrorType_t loop (float setpoint)  override;
private:
};
