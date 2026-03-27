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

namespace task {

void controller_task(void* args);

namespace controller {
	enum ControllerState_e : EventBits_t {
		IDLE    = 0b1 <<  0,
		WINDUP  = 0b1 <<  1,
		WINDOWN = 0b1 <<  2,
		CONTROL = 0b1 <<  3,
		ERROR   = 0b1 << 12
	};

	const StateSwitcher<ControllerState_e>& controller_switcher();

class ControllerTaskInterface {
public:
	struct config_params {
		QueueHandle_t setpoint;
		QueueHandle_t speed;
	};

	static ControllerTaskInterface                & get_instance();
	static StateSwitcher<ControllerState_e> const & get_switcher();

	void set_params(const config_params& params);
private:
	ControllerTaskInterface();
};

}

}
