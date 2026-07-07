#include "encoder.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_attr.h"

static const char *TAG = "encoder";

constexpr uint64_t IDLE_RESET_TIME_ms = 100000L;

// ISR global en IRAM
static void IRAM_ATTR encoder_isr_handler(void *arg) {
    Encoder *encoder = (Encoder *)arg;
    if (encoder) {
        encoder->handlePulse();
    }
}

Encoder::Encoder(gpio_num_t pin_a, gpio_num_t pin_b, uint16_t pulses_per_rev)
:
    _pin_a(pin_a),
    _pin_b(pin_b),
    _pulses_per_rev(pulses_per_rev),
    _last_pulse_time_us(0),
    _pulse_interval_us (0),
    _direction(1)
{ }

Encoder::~Encoder() {
    gpio_isr_handler_remove(_pin_a);
}

void Encoder::init() {
    gpio_config_t io_conf = {};
    io_conf.intr_type    = GPIO_INTR_POSEDGE;
    io_conf.mode         = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << _pin_a) | (1ULL << _pin_b);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en   = GPIO_PULLUP_ENABLE;

    ESP_ERROR_CHECK(gpio_config(&io_conf));

    esp_err_t install_result = gpio_install_isr_service(0);
    switch (install_result) {
        case ESP_OK:
            break;
        case ESP_ERR_INVALID_STATE:
            ESP_LOGW(TAG, "gpio_install_isr_service called multiple times");
            break;
        default:
            ESP_LOGE(TAG,
                "Fatal error encountered while installing ISR service:\n"
                "    ERRCODE: %s",
                esp_err_to_name(install_result)
            );
            break;
    }

    gpio_isr_handler_add(_pin_a, encoder_isr_handler, (void *)this);

    reset();
    ESP_LOGI(TAG, "Encoder init: pin_a=%d, pin_b=%d, PPR=%d", _pin_a, _pin_b, _pulses_per_rev);
}

void IRAM_ATTR Encoder::handlePulse() {
    uint64_t now = esp_timer_get_time();
    _pulse_interval_us  = now - _last_pulse_time_us;
    _last_pulse_time_us = now;
    _direction          = gpio_get_level(_pin_b) ? 1 : -1;
}

float Encoder::getRpm() const {
    uint64_t now_us     = esp_timer_get_time();
    uint64_t interval   = _pulse_interval_us;
    uint64_t last_pulse = _last_pulse_time_us;
    int      dir        = _direction;

    if ((now_us - last_pulse) > IDLE_RESET_TIME_ms) {
        return 0.0f;
    }
    if ( 0 == interval ) {
        return 0.0f;
    }

    float rpm = dir * (60.0f * 1e6f) / (interval * _pulses_per_rev);
    return rpm;
}

void Encoder::reset() {
    _last_pulse_time_us = esp_timer_get_time();
    _pulse_interval_us  = 0;
    _direction          = 1;
}

int32_t Encoder::getDirection() const {
    return _direction;
}

uint64_t Encoder::getLastPulseTime() const {
    return _last_pulse_time_us;
}
