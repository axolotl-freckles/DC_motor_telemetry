/**
 * @file StateTask.cpp
 * @author ACMAX (aavaloscorrales@gmail.com)
 * @brief
 * @version 0.1
 * @date 2026-03-28
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "tasks.hpp"

task::StateTask::StateTask()
:
	_frtos_task_h            (),
	_task_state_event_group_h()
{ }

task::StateTask::~StateTask() {
	if (_frtos_task_h) {
		vTaskDelete(_frtos_task_h);
		_frtos_task_h = nullptr;
	}
}

EventBits_t task::StateTask::get_state() {
	return xEventGroupGetBits(_task_state_event_group_h);
}
