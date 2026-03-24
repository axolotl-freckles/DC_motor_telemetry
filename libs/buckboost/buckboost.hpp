#ifndef BUCKBOOST_HPP
#define BUCKBOOST_HPP

#include "driver/ledc.h"
#include "driver/gpio.h"
#include <soc/gpio_sig_map.h>
#include <rom/gpio.h>

class BuckBoost {
public:
    BuckBoost(int pinSignal1, int pinSignal2, float vBat);
    void init();
    void setTargetVoltage(float targetVout);

private:
    void setDutyCycle(float duty);
    
    int pinSignal1_;
    int pinSignal2_;
    float V_BAT_;
    
    // PWM configuration
    static const uint32_t frequency = 100000; // 100 kHz
    static const uint8_t resolution = 8;      // 8-bit (0-255)
    static const int MAX_PWM_VAL = 255;
    static const uint8_t pwmChannel = 0;
    
    // Duty Cycle Constraints
    static constexpr float MIN_DUTY = 0.10f; // Minimum 10%
    static constexpr float MAX_DUTY = 0.90f; // Maximum 90%
};

#endif // BUCKBOOST_HPP