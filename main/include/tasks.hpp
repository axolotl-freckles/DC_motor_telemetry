/**
 * @file tasks.hpp
 * @author ACMAX (aaavaloscorrales@gmail.com)
 * @brief
 * @version 0.1
 * @date 2026-03-21
 *
 * @copyright Copyright (c) 2026
 *
 */
#pragma once

#include "freertos/FreeRTOS.h"

#include <functional>

namespace task {

void encoder_task   (void* args);
void controller_task(void* args);

template <typename state_enum_t>
class StateSwitcher {
public:
	using TransitionCallback_f =
		std::function<bool (state_enum_t from, state_enum_t to)>;
	using TransErrCallback_f   =
		std::function<void (state_enum_t from, state_enum_t to)>;

	StateSwitcher(
		EventGroupHandle_t   event_group_h,
		EventBits_t          state_mask              = ~(EventBits_t)0,
		TransitionCallback_f transition_callback     = TransitionCallback_f(),
		TransErrCallback_f   transition_err_callback = TransErrCallback_f  ()
	);

	inline void set_trans_callback    (TransitionCallback_f transition_callback)
	{ _trans_callback = transition_callback; }
	inline void set_trans_err_callback(TransErrCallback_f   trans_err_callback)
	{ _trans_callback = trans_err_callback; }
	inline EventBits_t  state_mask() const { return _state_mask; }
	inline EventBits_t& state_mask()       { return _state_mask; }

	bool update_state(state_enum_t new_state);

private:
	EventGroupHandle_t   _event_group_handle;
	EventBits_t          _state_mask;
	state_enum_t         _old_state;
	TransitionCallback_f _trans_callback;
	TransErrCallback_f   _trans_err_callback;

	StateSwitcher();
	StateSwitcher(StateSwitcher&);
	StateSwitcher(StateSwitcher&&);
};

namespace controller {
	enum ControllerState_e : EventBits_t {
		IDLE    = 0b1 <<  0,
		WINDUP  = 0b1 <<  1,
		WINDOWN = 0b1 <<  2,
		CONTROL = 0b1 <<  3,
		ERROR   = 0b1 << 12
	};
}

}

#include "tasks.t.hpp"
