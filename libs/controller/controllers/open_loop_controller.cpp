/**
 * @file open_loop_controller.cpp
 * @author ACMAX (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2026-06-22
 *
 * @copyright Copyright (c) 2026
 *
 */
#include "open_loop_controller.hpp"

#include <stdio.h>

#include "esp_timer.h"

#include "telemetry_task.hpp"
#include "num_calculus.hpp"

OpenLoop::OpenLoop () : Controller() {}

void OpenLoop::setup () {}

Controller::ErrorType_t OpenLoop::loop (float setpoint) {
	set_voltage(setpoint);
	return Controller::ErrorType_t::OK;
}