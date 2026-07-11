#include "bezierWindup.hpp"

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
    // Parámetros base del perfil original
    const float ORIGINAL_SHIFT = 0.0f;
    const float ORIGINAL_ADJUSTMENT = 0.5f;
    const float ORIGINAL_DURATION = 2.8f + ORIGINAL_SHIFT + ORIGINAL_ADJUSTMENT;

    // Usamos _en_setpoint como la velocidad máxima (KF_RAD_S) que escala el perfil completo
    const float kf_rad_s = _en_setpoint;

    // Factor de escala temporal dinámico usando el periodo del objeto (_period)
    const float scale_factor = _period / ORIGINAL_DURATION;

    // Marcadores de tiempo distribuidos proporcionalmente
    const float time_markers[6] = {
        (0.1f + ORIGINAL_SHIFT) * scale_factor,
        (0.5f + ORIGINAL_SHIFT) * scale_factor,
        (1.0f + ORIGINAL_SHIFT + ORIGINAL_ADJUSTMENT) * scale_factor,
        (1.7f + ORIGINAL_SHIFT + ORIGINAL_ADJUSTMENT) * scale_factor,
        (2.7f + ORIGINAL_SHIFT + ORIGINAL_ADJUSTMENT) * scale_factor,
        (2.8f + ORIGINAL_SHIFT + ORIGINAL_ADJUSTMENT) * scale_factor
    };

    float target_velocity_rad_s = 0.0f;
    int segment = 0;

    // Identificar el tramo en base al delta_time_s actual enviado
    for (int i = 0; i < 6; i++) {
        if (delta_time_s <= time_markers[i]) {
            segment = i + 1;
            break;
        }
    }
    if (segment == 0) {
        segment = 7;
    }

    // Máquina de estados matemática (Tus 7 segmentos originales intactos)
    switch (segment) {
        case 1:
            // CORRECCIÓN: Usamos el setpoint inicial asignado al objeto en lugar de 0.0f fijo
            target_velocity_rad_s = _st_setpoint; 
            break;

        case 2: {
            float k1_ramp1 = (delta_time_s - time_markers[0]) / (time_markers[1] - time_markers[0]);
            target_velocity_rad_s = kf_rad_s * calculate_bezier(k1_ramp1);
            break;
        }
        case 3:
            target_velocity_rad_s = kf_rad_s;
            break;

        case 4: {
            float k1_ramp2 = (delta_time_s - time_markers[2]) / (time_markers[3] - time_markers[2]);
            target_velocity_rad_s = kf_rad_s - kf_rad_s * 0.5f * calculate_bezier(k1_ramp2);
            break;
        }
        case 5:
            target_velocity_rad_s = kf_rad_s * 0.5f;
            break;

        case 6: {
            float k1_ramp3 = (delta_time_s - time_markers[4]) / (time_markers[5] - time_markers[4]);
            target_velocity_rad_s = (kf_rad_s * 0.5f) + kf_rad_s * 0.25f * calculate_bezier(k1_ramp3);
            break;
        }
        case 7:
        default:
            target_velocity_rad_s = kf_rad_s * (1.0f - 0.25f);
            break;
    }

    // Conversión a RPM utilizando tus factores de escala
    float final_rpm = target_velocity_rad_s * RAD_S_TO_RPM * RPM_SCALING_FACTOR;

    // Retorna el valor bruto (float) tal como lo pide la firma de Windup::step
    return final_rpm;
}