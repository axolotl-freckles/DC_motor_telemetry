/**
 * @file tasks.t.hpp
 * @author ACMAX (aavaloscorrales@gmail.com)
 * @brief
 * @version 0.1
 * @date 2026-03-21
 *
 * @copyright Copyright (c) 2026
 *
 */
#pragma once

#include "tasks.hpp"

template <typename state_enum_t>
task::StateSwitcher<state_enum_t>::StateSwitcher(
	EventGroupHandle_t   event_group_h,
	EventBits_t          state_mask,
	TransitionCallback_f transition_callback,
	TransErrCallback_f   transition_err_callback
) :
	_event_group_handle(event_group_h),
	_state_mask        (state_mask),
	_old_state         (),
	_trans_callback    (transition_callback),
	_trans_err_callback(transition_err_callback)
{ }

template <typename state_enum_t>
bool task::StateSwitcher<state_enum_t>::update_state(state_enum_t new_state) {
	EventBits_t curr_state    =   xEventGroupGetBits(_event_group_handle)
	                            & _state_mask;
	bool        transition_ok = true;

	xEventGroupClearBits(_event_group_handle, curr_state);
	if (_trans_callback) {
		transition_ok =  _trans_callback(_old_state, new_state);
	}

	if (transition_ok) {
		xEventGroupSetBits(_event_group_handle, new_state);
		_old_state = new_state;
		return transition_ok;
	}

	if (_trans_err_callback) {
		_trans_err_callback(_old_state, new_state);
	}

	xEventGroupSetBits(_event_group_handle, _old_state);
	return transition_ok;
}
