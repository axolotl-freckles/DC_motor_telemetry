/**
 * @file telemetry_task.hpp
 * @author ACMAX (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2026-07-01
 *
 * @copyright Copyright (c) 2026
 *
 */
#pragma once

#include "tasks.hpp"

namespace task {

namespace apply {

enum ApplyState_e : EventBits_t {
	IDLE     = 0b1 <<  0,
	APPLYING = 0b1 <<  1,
	ERROR    = 0b1 << 12
};

class ApplyTask : public StateTask {
public:
	struct config_params {
		QueueHandle_t voltage_queue_h = NULL;
	};

	static ApplyTask& get_instance();
	static QueueHandle_t createQueue(UBaseType_t len);

	void set_params(const config_params& params);

	esp_err_t start() override;
	esp_err_t stop () override;

	// EventBits_t get_state() override;
	// esp_err_t   wait_state(EventBits_t state, TickType_t timeout) override;

	virtual ~ApplyTask();
protected:
private:
	ApplyTask();
	ApplyTask(ApplyTask&);
	ApplyTask(ApplyTask&&);
};

} // namespace apply

} // namespace task