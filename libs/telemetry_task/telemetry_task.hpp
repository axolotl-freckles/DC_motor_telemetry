/**
 * @file telemetry_task.hpp
 * @author ACMAX (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2026-06-28
 *
 * @copyright Copyright (c) 2026
 *
 */
#pragma once

#include "tasks.hpp"


namespace task {

namespace telemetry {
constexpr size_t DATA_QUEUE_LEN = 100;

struct telemetry_data_t {
	float timestamp      = 0.0f;
	float setpoint       = 0.0f;
	float set_voltage    = 0.0f;
	float w_rad_s        = 0.0f;
	float I_amp          = 0.0f;
	float estimated_load = 0.0f;
};

class TelemetryTask : StateTask {
public:
	static TelemetryTask& get_instance();
	QueueHandle_t data_queue() { return _data_queue_h; }

	virtual esp_err_t start() override { return ESP_OK; }
	virtual esp_err_t stop () override { return ESP_OK; }

	virtual EventBits_t get_state() override {return 0;}

	virtual ~TelemetryTask() {}
protected:
private:
	StaticEventGroup_t _telemetry_state_event_group;
	QueueHandle_t      _data_queue_h;

	TelemetryTask();
};

} // namespace telemetry

} // namespace task