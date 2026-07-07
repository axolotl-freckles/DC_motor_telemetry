#pragma once

#include <cstdint>
#include "driver/gpio.h"

class Encoder {
public:
    Encoder(
        gpio_num_t pin_a,
        gpio_num_t pin_b,
        uint16_t pulses_per_rev = 1600
    );
    ~Encoder();

    void init();
    void reset();

    float    getRpm() const;
    int32_t  getDirection() const;
    uint64_t getLastPulseTime() const;

    void handlePulse(); // Called from ISR
private:
    gpio_num_t _pin_a;
    gpio_num_t _pin_b;
    uint16_t   _pulses_per_rev;

    volatile uint64_t _last_pulse_time_us;
    volatile uint64_t _pulse_interval_us;
    volatile int      _direction;
};
