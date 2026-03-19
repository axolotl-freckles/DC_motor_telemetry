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

enum FluxSpeed_t {
	FREQUENCY, ANGULAR_SPEED
};

struct FluxSpeed {
	float value;
	FluxSpeed_t type;
};

struct ControlPoint {
	float amplitude;
	FluxSpeed flux_speed;

	inline bool operator < (const ControlPoint &_other) {
		assert(flux_speed.type == _other.flux_speed.type);
		return
			   (       amplitude < _other.amplitude       )
			|| (flux_speed.value < _other.flux_speed.value);
	}
	inline bool operator > (const ControlPoint &_other) {
		assert(flux_speed.type == _other.flux_speed.type);
		return
			   (       amplitude > _other.amplitude       )
			|| (flux_speed.value > _other.flux_speed.value);
	}
};

} // namespace control
