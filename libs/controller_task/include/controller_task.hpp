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

	const StateSwitcher<ControllerState_e>& controller_switcher();

class ControllerTask {
public:
	struct config_params {
		QueueHandle_t setpoint_qh;
		QueueHandle_t speed_qh;
		QueueHandle_t control_signal_qh;
	};

	static ControllerTask            & get_instance();
	StateSwitcher<ControllerState_e> & get_switcher();

	void set_params(const config_params& params);
private:
	/* Task management variables */
	TaskHandle_t                      _frtos_task_h;
	EventGroupHandle_t                _controller_state_event_group_h;
	StateSwitcher<ControllerState_e> *_transition_handler;
	/* Runtime variables */
	Controller                       *_controller;
	/* Message interface variables */
	QueueHandle_t                     _setpoint_qh;
	QueueHandle_t                     _speed_qh;
	QueueHandle_t                     _csignal_qh;

	ControllerTask();
};

}

}
