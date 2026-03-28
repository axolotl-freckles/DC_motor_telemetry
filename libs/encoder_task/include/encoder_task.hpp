/**
 * @file encoder_task.hpp
 * @author ACMAX (aavaloscorrales@gmail.com)
 * @brief
 * @version 0.1
 * @date 2026-03-26
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "tasks.hpp"

namespace task{

namespace encoder {
	enum EncoderState_e : EventBits_t {
		IDLE     = 0b1 << 0,
		SAMPLING = 0b1 << 1,
		ERROR    = 0b1 << 12
	};

class EncoderTask {
public:
	struct config_params {
		QueueHandle_t speed_qh;
	};

	static EncoderTask            & get_instance();
	StateSwitcher<EncoderState_e> & get_switcher();

	void set_params(const config_params& params);
private:
	/* Task management variables */
	TaskHandle_t                   _frtos_task_h;
	StaticEventGroup_t             _encoder_state_event_group;
	EventGroupHandle_t             _encoder_state_event_group_h;
	StateSwitcher<EncoderState_e> *_transition_handler;
	/* Runtime variables */
	/* Message interface variables */
	QueueHandle_t                  _speed_qh;

	EncoderTask();
};


}

}
