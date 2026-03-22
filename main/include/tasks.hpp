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

#include "freertos/FreeRTOS.h"

namespace task {

void encoder_task(void* args);
void controller_task(void* args);

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
