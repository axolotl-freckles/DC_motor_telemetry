


#pragma once

#include "windup.hpp"

class BezierWindup : public Windup {
public:
    // Implementación de los métodos virtuales puros de la clase padre
    float step(float delta_time_s) const override;
    
    void set_st_setpoint(float setpoint) override;
    void set_en_setpoint(float setpoint) override;
    
    float get_en_setpoint() const override;

    // Métodos específicos de esta clase
    inline void set_period(float period) { _period = period; }
    inline float get_st_setpoint() const { return _st_setpoint; }

    // Constructor y destructor
    BezierWindup(float period_s, float start_setpoint, float end_setpoint);
    virtual ~BezierWindup() { }

private:
    float _st_setpoint;
    float _en_setpoint;

    // Constantes matemáticas internas para Bézier
    static constexpr float R1 = 252.0f, R2 = 1050.0f, R3 = 1800.0f;
    static constexpr float R4 = 1575.0f, R5 = 700.0f, R6 = 126.0f;

    // Función auxiliar matemática
    float calculate_bezier(float k1) const;
};