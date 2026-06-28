/**
 * @file controller.cpp
 * @author ACMAX (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2025-05-16
 *
 * @copyright Copyright (c) 2025
 *
 */
#include "controller.hpp"

#include <limits>

// TODO: remove this constant and define it somewhere else, this was just to
//       test the library import
constexpr float FIRMWARE_TICK_INTERVAL_ms = 100;

Controller::Controller() :
	  windup (nullptr)
	, winddown(nullptr)
{
	this->control_point.voltage = 0.0f;
}

void Controller::set_voltage (const float voltage) {
	this->control_point.voltage = voltage;
}
void Controller::set_windup  (const Windup   *windup  ) {
	this->windup   = windup;
}
void Controller::set_winddown(const Winddown *winddown) {
	this->winddown = winddown;
}

float Controller::read_pcb_current(void) {
	return std::numeric_limits<float>::signaling_NaN();
}
float Controller::read_source_voltage(void) {
	return std::numeric_limits<float>::signaling_NaN();
}

float Controller::get_sample_time_s(void) {
	return FIRMWARE_TICK_INTERVAL_ms * 1e-3;
}
float Controller::get_sample_frequency_hz(void) {
	return 1.0 / (FIRMWARE_TICK_INTERVAL_ms * 1.e-3f);
}