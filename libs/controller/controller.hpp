/**
 * @file controller.hpp
 * @author ACMAX (aavaloscorrales@gmail.com)
 * @brief
 * @version 0.1
 * @date 2025-02-20
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once

#include "controller_types.hpp"
#include "windup.hpp"
#include "winddown.hpp"

class Controller {
public:
	Controller();
	virtual ~Controller() { }

/*
 * OVERLOAD THEESE TWO FUNCTIONS FOR YOUR CONTROLLER
 * setup() -> runs after windup, use to set the internal variables in your
 *          controller
 * loop()  -> runs every kernel tick. put your controller logic here
 */
	virtual void setup() = 0;
	virtual void loop()  = 0;

	void set_voltage (const float voltage);
	void set_windup  (const Windup   *windup);
	void set_winddown(const Winddown *winddown);

	static float read_pcb_current(void);
	static float read_source_voltage(void);

	static float get_sample_time_s(void);
	static float get_sample_frequency_hz(void);

	/*#### Kernel interface ####*/
	inline const control::ControlPoint get_control_point() {
		return control_point;
	}
	inline const Windup   *get_windup()   { return windup;  }
	inline const Winddown *get_winddown() { return winddown; }

private:
	control::ControlPoint control_point;
	const Windup   *windup;
	const Winddown *winddown;
};
