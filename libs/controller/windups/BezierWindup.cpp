#include "BezierWindup.hpp"

BezierWindup::BezierWindup(float period_s, float start_setpoint, float end_setpoint)
    : Windup(period_s)
    , _st_setpoint(start_setpoint)
    , _en_setpoint(end_setpoint)
{ }

void BezierWindup::set_st_setpoint(float setpoint) {
    _st_setpoint = setpoint;
}

void BezierWindup::set_en_setpoint(float setpoint) {
    _en_setpoint = setpoint;
}

float BezierWindup::get_en_setpoint() const {
    return _en_setpoint;
}

float BezierWindup::calculate_bezier(float k1) const {
    float k1_pow_2 = k1 * k1;
    float k1_pow_3 = k1_pow_2 * k1;
    float k1_pow_4 = k1_pow_3 * k1;
    float k1_pow_5 = k1_pow_4 * k1;

    return k1_pow_5 * (R1 - (R2 * k1) + (R3 * k1_pow_2) - (R4 * k1_pow_3) + (R5 * k1_pow_4) - (R6 * k1_pow_5));
}

float BezierWindup::step(float delta_time_s) const {
    float k1_ramp1 = delta_time_s / _period;
    return _st_setpoint + (_en_setpoint-_st_setpoint)*calculate_bezier(k1_ramp1);
}