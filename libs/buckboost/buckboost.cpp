#include "buckboost.hpp"
#include <cstdio>

BuckBoost::BuckBoost(int pinSignal1, int pinSignal2, float vBat)
    : pinSignal1_(pinSignal1), pinSignal2_(pinSignal2), V_BAT_(vBat) {}

void BuckBoost::init() {
    // Configure LEDC timer
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = frequency,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer_conf);

    // Configure LEDC channel
    ledc_channel_config_t channel_conf = {
        .gpio_num = pinSignal1_,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&channel_conf);

    // Configure Pin 2 as output
    gpio_set_direction((gpio_num_t)pinSignal2_, GPIO_MODE_OUTPUT);

    // Route the inverted internal signal to Pin 2
    gpio_matrix_out(pinSignal2_, LEDC_LS_SIG_OUT0_IDX + pwmChannel, true, false);
}

void BuckBoost::setTargetVoltage(float targetVout) {
    // Prevent negative voltage
    if (targetVout < 0.0f) targetVout = 0.0f;

    // Calculate duty cycle: D = Vout / (Vout + Vbat)
    float calculatedD = targetVout / (targetVout + V_BAT_);
    
    printf("\n--- New Target Voltage: %.2f V ---\n", targetVout);

    // Apply the duty cycle
    setDutyCycle(calculatedD);
}

void BuckBoost::setDutyCycle(float duty) {
    // Clamp duty cycle
    if (duty > MAX_DUTY) {
        duty = MAX_DUTY;
        printf("WARNING: Target voltage too high. Clamping D to 90%%!\n");
    } else if (duty < MIN_DUTY) {
        duty = MIN_DUTY;
        printf("WARNING: Target voltage too low. Clamping D to 10%%!\n");
    }

    // Calculate duty value
    uint32_t dutyValue = (uint32_t)(duty * MAX_PWM_VAL);

    // Set duty and update
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, dutyValue);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    
    printf("Calculated D: %.2f%%\n", duty * 100.0f);
}