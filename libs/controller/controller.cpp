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

#include "sampler_task.hpp"
#include "globals.hpp"

Controller::Controller() :
	  windup (nullptr)
	, winddown(nullptr)
{
	this->control_point.voltage = 0.0f;
}

void Controller::set_voltage (const float voltage) {
	this->control_point.voltage = voltage;
}
void Controller::set_windup  (Windup   *windup  ) {
	this->windup   = windup;
}
void Controller::set_winddown(Winddown *winddown) {
	this->winddown = winddown;
}

float Controller::read_current(void) {
	return task::sampler::SamplerTask::get_instance().current_TI();
}
float Controller::read_voltage(void) {
	return task::sampler::SamplerTask::get_instance().current_Volt();
}
float Controller::read_speed_rad_s(void) {
	return task::sampler::SamplerTask::get_instance().current_w();
}
float Controller::estimated_load_nm(void) {
	return task::sampler::SamplerTask::get_instance().current_TL();
}

float Controller::get_sample_time_s(void) {
	return SAMPLE_TIME_s;
}
float Controller::get_sample_frequency_hz(void) {
	return 1.0f / SAMPLE_TIME_s;
}