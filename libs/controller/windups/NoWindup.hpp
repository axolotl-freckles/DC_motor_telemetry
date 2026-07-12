/**
 * @file NoWindup.hpp
 * @author ACMAX (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2026-07-10
 *
 * @copyright Copyright (c) 2026
 *
 */
#pragma once

#include "windup.hpp"

class NoWindup : public Windup {
public:
	float step(float delta_time_s) const override;

	inline void set_st_setpoint(float setpoint) override { }
	inline void set_en_setpoint(float setpoint) override {_en_setpoint = setpoint;}

	inline float get_en_setpoint() const override { return _en_setpoint; }

	NoWindup();
	NoWindup(float end_setpoint);
	virtual ~NoWindup() {}
protected:
private:
	float _en_setpoint = 0.0f;
};
