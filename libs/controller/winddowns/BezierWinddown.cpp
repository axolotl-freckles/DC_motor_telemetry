

#include "BezierWinddown.hpp"

BezierWinddown::BezierWinddown(float period_s, float start_setpoint, float end_setpoint)
    : Winddown(period_s)
    , _st_setpoint(start_setpoint)
    , _en_setpoint(end_setpoint)
{ }

void BezierWinddown::set_st_setpoint(float setpoint) {
    _st_setpoint = setpoint;
}

void BezierWinddown::set_en_setpoint(float setpoint) {
    _en_setpoint = setpoint;
}

float BezierWinddown::get_en_setpoint() const {
    return _en_setpoint;
}

float BezierWinddown::calculate_bezier(float k1) const {
    float k1_pow_2 = k1 * k1;
    float k1_pow_3 = k1_pow_2 * k1;
    float k1_pow_4 = k1_pow_3 * k1;
    float k1_pow_5 = k1_pow_4 * k1;

    return k1_pow_5 * (R1 - (R2 * k1) + (R3 * k1_pow_2) - (R4 * k1_pow_3) + (R5 * k1_pow_4) - (R6 * k1_pow_5));
}

float BezierWinddown::step(float delta_time_s) const {
    float k1_ramp1 = delta_time_s / _period;
    return _st_setpoint + (_en_setpoint - _st_setpoint)*calculate_bezier(k1_ramp1);
}