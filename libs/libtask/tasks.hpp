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

#include <functional>

#include "esp_err.h"

#include "freertos/FreeRTOS.h"

namespace task {

class StateTask {
public:
	virtual esp_err_t start() = 0;
	virtual esp_err_t stop()  = 0;

	virtual EventBits_t get_state();

	StateTask();
	virtual ~StateTask();
protected:
	TaskHandle_t       _frtos_task_h             = nullptr;
	EventGroupHandle_t _task_state_event_group_h = nullptr;
};

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
	EventGroupHandle_t   _event_group_handle = nullptr;
	EventBits_t          _state_mask         = 0;
	state_enum_t         _old_state;
	TransitionCallback_f _trans_callback;
	TransErrCallback_f   _trans_err_callback;

	StateSwitcher();
	StateSwitcher(StateSwitcher&);
	StateSwitcher(StateSwitcher&&);
};

}

#include "tasks.t.hpp"
