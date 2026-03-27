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

void encoder_task(void* args);

namespace encoder {
	enum EncoderState_e : EventBits_t {
		IDLE     = 0b1 << 0,
		SAMPLING = 0b1 << 1,
		ERROR    = 0b1 << 12
	};

	const StateSwitcher<EncoderState_e>& encoder_switcher();
}

}
