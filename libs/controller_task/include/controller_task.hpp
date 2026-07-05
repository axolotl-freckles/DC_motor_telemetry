/**
 * @file controller_task.hpp
 * @author ACMAX (aavaloscorrales@gmail.com)
 * @brief
 * @version 0.1
 * @date 2026-03-26
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "tasks.hpp"

#include "controller.hpp"

namespace task {

namespace controller {
	enum ControllerState_e : EventBits_t {
		IDLE    = 0b1 <<  0,
		WINDUP  = 0b1 <<  1,
		WINDOWN = 0b1 <<  2,
		CONTROL = 0b1 <<  3,
		ERROR   = 0b1 << 12
	};

class ControllerTask : public StateTask {
public:
	struct config_params {
		QueueHandle_t setpoint_qh;
		QueueHandle_t speed_qh;
		QueueHandle_t control_signal_qh;
	};

	static ControllerTask& get_instance();

	void set_params(const config_params& params);

	esp_err_t start() override;
	esp_err_t stop()  override;

	EventBits_t get_state() override;
	esp_err_t   wait_state(EventBits_t state, TickType_t timeout) override;

	virtual ~ControllerTask();
private:
	ControllerTask();

	esp_err_t wait_sync();
};

}

}
