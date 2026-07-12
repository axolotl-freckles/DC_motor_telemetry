/**
 * @file monitor_input.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2026-07-12
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#pragma once

#include "tasks.hpp"

namespace task {
namespace monitor_in {

enum MonitorInputState_e : EventBits_t {
	IDLE      = 0b1 <<  0,
	RECEIVING = 0b1 <<  1,
	ERROR     = 0b1 << 12
};

class MonitorInput : public StateTask {
public:
	virtual esp_err_t start() override;
	virtual esp_err_t stop()  override;

	void set_setpoint_qh(QueueHandle_t setpoint_qh);

	static bool try_read_setpoint_rpm(float &setpoint_rpm_out);
	static MonitorInput& get_instance();

	virtual ~MonitorInput() {};
private:
	MonitorInput();
};

} // namespace monitor_in

} // namespace task
