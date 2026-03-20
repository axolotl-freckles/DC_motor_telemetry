/**
 * @file controller_types.hpp
 * @author ACMAX (aavaloscorrales@gmail.com)
 * @brief
 * @version 0.1
 * @date 2025-08-13
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once

#include <cassert>

namespace control {

struct ControlPoint {
	float voltage;

	inline bool operator < (const ControlPoint &_other) {
		return voltage < _other.voltage;
	}
	inline bool operator > (const ControlPoint &_other) {
		return voltage > _other.voltage;
	}
};

} // namespace control
