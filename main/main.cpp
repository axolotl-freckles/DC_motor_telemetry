#include <stdio.h>
#include <algorithm>

#include "esp_log.h"

#include "freertos/FreeRTOS.h"

#include "driver/ledc.h"
#include "driver/gpio.h"

/* ============================================ PWM Configuration Constants ============================================ */
static constexpr gpio_num_t BOOST_PIN = GPIO_NUM_21;
static constexpr gpio_num_t BUCK_PIN  = GPIO_NUM_18;
static constexpr ledc_channel_t BOOST_CH = LEDC_CHANNEL_0;
static constexpr ledc_channel_t BUCK_CH  = LEDC_CHANNEL_1;
static constexpr ledc_timer_t LEDC_TIMER = LEDC_TIMER_0;
static constexpr uint32_t LEDC_FREQUENCY = 100000;  // 100 kHz
static constexpr ledc_timer_bit_t LEDC_RESOLUTION = LEDC_TIMER_8_BIT;
static constexpr uint32_t PWM_RESOLUTION_MAX = 255;

/* ============================================ Converter Parameters ============================================ */
static constexpr float VBAT = 12.0f;      // Battery voltage in volts (constant)
static constexpr float D_MIN = 0.1f;      // Minimum duty cycle
static constexpr float D_MAX = 0.9f;      // Maximum duty cycle
static constexpr const char *LOG_TAG = "power_control";

/* ============================================ Initialize PWM ============================================ */
static void init_pwm(void) {
	// Configure LEDC timer
	ledc_timer_config_t timer_cfg = {
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.duty_resolution = LEDC_RESOLUTION,
		.timer_num = LEDC_TIMER,
		.freq_hz = LEDC_FREQUENCY,
		.clk_cfg = LEDC_AUTO_CLK
	};
	ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

	// Configure BOOST channel (GPIO21)
	ledc_channel_config_t boost_cfg = {
		.gpio_num = BOOST_PIN,
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.channel = BOOST_CH,
		.timer_sel = LEDC_TIMER,
		.duty = 0,
		.hpoint = 0,
		.flags = {.output_invert = 0}
	};
	ESP_ERROR_CHECK(ledc_channel_config(&boost_cfg));

	// Configure BUCK channel (GPIO18)
	ledc_channel_config_t buck_cfg = {
		.gpio_num = BUCK_PIN,
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.channel = BUCK_CH,
		.timer_sel = LEDC_TIMER,
		.duty = 0,
		.hpoint = 0,
		.flags = {.output_invert = 0}
	};
	ESP_ERROR_CHECK(ledc_channel_config(&buck_cfg));

	ESP_LOGI(LOG_TAG, "PWM initialized on GPIO21 (BOOST) and GPIO18 (BUCK)");
}

/* ============================================ Calculate Duty Cycle ============================================ */
static float calculate_duty_cycle(float vout) {
	// D = Vout / (Vout + Vbat)
	if (vout < 0.0f) vout = 0.0f;
	
	float D = vout / (vout + VBAT);
	
	// Clamp D between D_MIN and D_MAX
	D = std::max(D_MIN, std::min(D, D_MAX));
	
	ESP_LOGD(LOG_TAG, "Vout=%.2fV, D=%.3f, Boost=(1-D)=%.3f, Buck=D=%.3f", vout, D, 1.0f - D, D);
	
	return D;
}

/* ============================================ Power Control Task ============================================ */
static void power_control_task(void *arg) {
	const float vout = 6.0f;
	const TickType_t update_period = pdMS_TO_TICKS(1000);

	ESP_LOGI(LOG_TAG, "Power control test started: constant Vout=%.2fV, update every 1000ms", vout);

	while (true) {
		float D = calculate_duty_cycle(vout);

		uint32_t boost_duty = (uint32_t)((1.0f - D) * PWM_RESOLUTION_MAX);
		uint32_t buck_duty  = (uint32_t)(D * PWM_RESOLUTION_MAX);
		uint32_t buck_hpoint = boost_duty;

		ESP_ERROR_CHECK(ledc_set_duty_with_hpoint(LEDC_LOW_SPEED_MODE, BOOST_CH, boost_duty, 0));
		ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, BOOST_CH));
		ESP_ERROR_CHECK(ledc_set_duty_with_hpoint(LEDC_LOW_SPEED_MODE, BUCK_CH, buck_duty, buck_hpoint));
		ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, BUCK_CH));

		ESP_LOGI(LOG_TAG, "TEST Vout=%.2fV | D=%.3f | BOOST_PWM=%d (%.1f%%) [0..%d] | BUCK_PWM=%d (%.1f%%) [%d..%d]",
			 vout, D,
			 boost_duty, (boost_duty * 100.0f / PWM_RESOLUTION_MAX),
			 boost_duty,
			 buck_duty, (buck_duty * 100.0f / PWM_RESOLUTION_MAX),
			 buck_hpoint, buck_hpoint + buck_duty);

		vTaskDelay(update_period);
	}
}

extern "C" void app_main(void)
{
	// Initialize PWM
	init_pwm();

	// Create power control test task with constant 6V setpoint
	xTaskCreate(
		power_control_task,
		"power_control_task",
		2048,
		nullptr,
		2,
		nullptr
	);

	while (true) {
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

