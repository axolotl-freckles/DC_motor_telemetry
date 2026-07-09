#include <stdio.h>
#include <algorithm>

#include "esp_heap_trace.h"
#include "esp_log.h"
#include "driver/ledc.h"

#include "freertos/FreeRTOS.h"

#include "driver/ledc.h"
#include "driver/gpio.h"

#include "controller.hpp"
#include "controllers/pid_controller.hpp"
#include "filters.hpp"

#include "tasks.hpp"
#include "controller_task.hpp"
#include "sampler_task.hpp"
#include "apply_task.hpp"

using task::controller::ControllerTask;
using task::controller::ControllerState_e;
using task::sampler::SamplerTask;
using task::apply::ApplyTask;

static constexpr gpio_num_t BOOST_PIN = GPIO_NUM_21;
static constexpr gpio_num_t BUCK_PIN  = GPIO_NUM_18;
static constexpr ledc_channel_t BOOST_CH = LEDC_CHANNEL_0;
static constexpr ledc_channel_t BUCK_CH  = LEDC_CHANNEL_1;
static constexpr ledc_timer_t LEDC_TIMER = LEDC_TIMER_0;
static constexpr uint32_t LEDC_FREQUENCY = 100000;  // 100 kHz
static constexpr ledc_timer_bit_t LEDC_RESOLUTION = LEDC_TIMER_8_BIT;
static constexpr uint32_t PWM_RESOLUTION_MAX = 255;
static constexpr uint32_t PWM_DEAD_TIME = 2; // dead time in timer counts
// Set these to 1 to invert the physical output polarity if needed for your hardware
static constexpr int BOOST_OUTPUT_INVERT = 0;
static constexpr int BUCK_OUTPUT_INVERT  = 0;

static constexpr float VBAT = 12.0f;      // Battery voltage in volts (constant)
static constexpr float VOUT_MAX = 24.0f;   // Maximum expected control voltage for PWM mapping
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

	// Install LEDC fade service required by ledc_set_duty_and_update
	ESP_ERROR_CHECK(ledc_fade_func_install(0));

	// Configure BOOST channel (GPIO21)
	ledc_channel_config_t boost_cfg = {
		.gpio_num = BOOST_PIN,
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.channel = BOOST_CH,
		.timer_sel = LEDC_TIMER,
		.duty = 0,
		.hpoint = 0,
		.flags = {.output_invert = BOOST_OUTPUT_INVERT}
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
		.flags = {.output_invert = BUCK_OUTPUT_INVERT}
	};
	ESP_ERROR_CHECK(ledc_channel_config(&buck_cfg));

	/* Ensure outputs start at 0 duty */
	ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, BOOST_CH, 0));
	ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, BOOST_CH));
	ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, BUCK_CH, 0));
	ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, BUCK_CH));

	ESP_LOGI(LOG_TAG, "PWM initialized on GPIO21 (BOOST) and GPIO18 (BUCK)");
}

/* ============================================ Calculate Duty Cycle ============================================ */
static float calculate_duty_cycle(float vout) {
	// D = Vout / (Vout + VBAT)
	if (vout < 0.0f) {
		vout = 0.0f;
	} else if (vout > VOUT_MAX) {
		vout = VOUT_MAX;
	}

	float D = vout / (vout + VBAT);

	// Clamp D between D_MIN and D_MAX
	D = std::max(D_MIN, std::min(D, D_MAX));

	return D;
}

/* ============================================ Power Control Task ============================================ */
static void power_control_task(void *arg) {
	QueueHandle_t cpoint_qh = (QueueHandle_t)arg;
	float vout = 0.0f;
	float last_vout = 0.0f;
	float last_logged_vout = -1.0f;
	uint32_t last_logged_boost = UINT32_MAX;
	uint32_t last_logged_buck = UINT32_MAX;
	TickType_t last_log_tick = 0;
	bool have_last_vout = false;
	bool warn_no_signal = false;

	ESP_LOGI(LOG_TAG, "Power control task started");

	while (true) {
		bool received = false;
		float new_vout = 0.0f;

		if (xQueueReceive(cpoint_qh, &new_vout, pdMS_TO_TICKS(100)) == pdTRUE) {
			vout = new_vout;
			have_last_vout = true;
			last_vout = vout;
			received = true;
			warn_no_signal = false;
		} else if (have_last_vout) {
			vout = last_vout;
			if (!warn_no_signal && xTaskGetTickCount() - last_log_tick > pdMS_TO_TICKS(1000)) {
				ESP_LOGW(LOG_TAG, "No new control point in 1s, holding Vout=%.3f", vout);
				warn_no_signal = true;
			}
		} else {
			vout = 0.0f;
		}

		float D = calculate_duty_cycle(vout);

		uint32_t boost_duty = (uint32_t)((1.0f - D) * PWM_RESOLUTION_MAX);
		uint32_t buck_duty = (uint32_t)(D * PWM_RESOLUTION_MAX);
		uint32_t buck_hpoint = std::min(boost_duty + PWM_DEAD_TIME, PWM_RESOLUTION_MAX - 1u);

		if (boost_duty + buck_duty + PWM_DEAD_TIME > PWM_RESOLUTION_MAX) {
			buck_duty = (PWM_RESOLUTION_MAX > boost_duty + PWM_DEAD_TIME) ?
				(PWM_RESOLUTION_MAX - boost_duty - PWM_DEAD_TIME) : 0;
		}

		ESP_ERROR_CHECK(ledc_set_duty_with_hpoint(LEDC_LOW_SPEED_MODE, BOOST_CH, boost_duty, 0));
		ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, BOOST_CH));

		ESP_ERROR_CHECK(ledc_set_duty_with_hpoint(LEDC_LOW_SPEED_MODE, BUCK_CH, buck_duty, buck_hpoint));
		ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, BUCK_CH));

		TickType_t now = xTaskGetTickCount();
		bool should_log = false;
		if (received) {
			should_log = true;
		}
		if (vout > last_logged_vout + 0.05f || vout < last_logged_vout - 0.05f) {
			should_log = true;
		}
		if (boost_duty != last_logged_boost || buck_duty != last_logged_buck) {
			should_log = true;
		}
		if (now - last_log_tick > pdMS_TO_TICKS(2000)) {
			should_log = true;
		}

		if (should_log) {
			ESP_LOGI(LOG_TAG, "PWM: Vout=%.3f D=%.3f boost=%u buck=%u hpoint=%u", vout, D, boost_duty, buck_duty, buck_hpoint);
			last_logged_vout = vout;
			last_logged_boost = boost_duty;
			last_logged_buck = buck_duty;
			last_log_tick = now;
		}

		vTaskDelay(pdMS_TO_TICKS(10));
	}
}

extern "C" void app_main(void)
{
	ControllerTask *controller_task = &ControllerTask::get_instance();
	SamplerTask    *sampler_task    = &SamplerTask::get_instance();
	ApplyTask      *apply_task      = &ApplyTask::get_instance();
	ledc_fade_func_install(0);

	QueueHandle_t setpoint_qh = xQueueCreate(1, sizeof(float));
	QueueHandle_t speed_qh    = xQueueCreate(1, sizeof(float));
	QueueHandle_t cpoint_qh   = xQueueCreate(1, sizeof(float));
	ControllerTask::config_params controller_config = {
		.setpoint_qh       = setpoint_qh,
		.speed_qh          = speed_qh,
		.control_signal_qh = cpoint_qh
	};
	SamplerTask::config_params sampler_config = {
		.speed_qh = speed_qh
	};
	ApplyTask::config_params apply_config = {
		.voltage_queue_h = cpoint_qh
	};
	controller_task->set_params(controller_config);
	sampler_task   ->set_params(sampler_config);
	apply_task     ->set_params(apply_config);

	// Initialize PWM
	init_pwm();

	xTaskCreate(
		power_control_task,
		"power_control_task",
		4096,
		cpoint_qh,
		2,
		nullptr
	);

	// Original startup sequence: wait, start controller (windup), run, then stop
	vTaskDelay(pdMS_TO_TICKS(3000));
	controller_task.start();
	controller_task.wait_state(ControllerState_e::CONTROL, portMAX_DELAY);

	vTaskDelay(pdMS_TO_TICKS(12000));
	controller_task->stop();

	while (true) {
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}

