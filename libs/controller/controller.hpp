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
	enum ErrorType_t {
		OK            = 0,
		GENERIC_ERROR,
		NAN_RESULT,
	};
	Controller();
	virtual ~Controller() { }

/*
 * OVERLOAD THEESE TWO FUNCTIONS FOR YOUR CONTROLLER
 * setup() -> runs after windup, use to set the internal variables in your
 *          controller
 * loop()  -> runs every kernel tick. put your controller logic here
 */
	virtual void        setup()                = 0;
	virtual ErrorType_t loop (float setpoint)  = 0;

	void set_voltage (const float voltage);
	void set_windup  (Windup     *windup);
	void set_winddown(Winddown   *winddown);

	static float read_current(void);
	static float read_voltage(void);
	static float read_speed_rad_s(void);
	static float estimated_load_nm(void);

	static float get_sample_time_s(void);
	static float get_sample_frequency_hz(void);

	/*#### Kernel interface ####*/
	inline const control::ControlPoint get_control_point() {
		return control_point;
	}
	inline const Windup   *get_windup()   const { return windup;   }
	inline const Winddown *get_winddown() const { return winddown; }
	inline       Windup   *get_windup()         { return windup;   }
	inline       Winddown *get_winddown()       { return winddown; }

private:
	control::ControlPoint control_point;
	Windup   *windup   = nullptr;
	Winddown *winddown = nullptr;
};
