/**
 * @file measurement.main.cpp
 * @author ACMAX (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2026-08-30
 *
 * @copyright Copyright (c) 2026
 *
 */
#include "sdkconfig.h"

#ifdef CONFIG_CONTROLLER_BUILD_MODE_FUNCTION_MEASUREMENT

#include <cstdio>
#include <cstdint>
#include <cmath>

#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "driver/ledc.h"
#include "driver/uart.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "globals.hpp"
#include "sampler_task.hpp"
#include "encoder.hpp"
#include "dc_plant.hpp"
#include "controller.hpp"
#include "controllers/ideal_control_law.hpp"
#include "controllers/pid_controller.hpp"

using namespace DCPlant;
using task::sampler::SamplerTask;

constexpr size_t MEASUREMENT_AMOUNT = 500;
constexpr char LOG_TAG[] = "function stats";

struct stadistics
{
	float max     = 0.0f;
	float min     = 0.0f;
	float average = 0.0f;
	float std_dev = 0.0f;
};

enum experiment : int {
	HANDLE_PULSE = 0,
	FXD_TO_REPR,
	FXD_FROM_REPR,
	FXD_MUL,
	OBSERVER_STEP,
	IDEAL_CONTROL_LOOP,
	PID_CONTROL_LOOP,
	MAX
};

int64_t times[MEASUREMENT_AMOUNT] = { 0 };

static void get_stadistics(const int64_t *const values, const size_t size, stadistics &stats);
static void print_stats(const char* title, const stadistics &stats);

inline float int64_us_to_s (const int64_t us) {
	//return 1.0e-6f*us;
	return (float)us;
}

#define EXP_PRINT_STATS(exp,stat_arr) print_stats(#exp,stat_arr[exp])

float error_func(float _setpoint) {
	return _setpoint - Controller::read_speed_rad_s();
};

extern "C" void app_main(void) {
	SamplerTask    &sampler_task    = SamplerTask::get_instance();
	Encoder test_encoder(sampler_task.get_encoder());
	DCMotorObserver_64 test_observer(
		SAMPLE_PARAMS, SAMPLE_OBS_PRMS, MODEL_SIM_TIME_s
	);
	IdealControlLaw    ideal_control(
		3.1758f, /* K1 */
		0.4152f, /* K2 */
		0.0975f, /* Ki */
		0.4560f  /* Ku */
	);
	PID                 pid_control(error_func, 0.8f, 0.2f, 0.0f);
	Controller         *test_controller = nullptr;

	int64_t *const st = times;
	int64_t *const en = times + MEASUREMENT_AMOUNT;
	int64_t *idx = nullptr;
	int64_t st_time = 0;
	int64_t en_time = 0;
	stadistics stats[experiment::MAX];

	ESP_LOGI(LOG_TAG, "Init finished");

	/* TEST ENCODER       */
	ESP_LOGI(LOG_TAG, "Encoder tests");
	test_encoder.reset();
	idx = st;
	while ( idx < en ) {
		st_time = esp_timer_get_time();
		test_encoder.handlePulse();
		en_time = esp_timer_get_time();
		*(idx++) = en_time - st_time;
	}
	get_stadistics(st, MEASUREMENT_AMOUNT, stats[experiment::HANDLE_PULSE]);

	/* TEST OBSERVER      */
	ESP_LOGI(LOG_TAG, "Observer tests");
	int64_t mock_voltage = DCMotorObserver_64::to_repr(20.0f);
	int64_t mock_speed   = DCMotorObserver_64::to_repr(256.0f);
	test_observer.reset();
	idx = st;
	while ( idx < en ) {
		st_time = esp_timer_get_time();
		(void)test_observer.step(mock_voltage, mock_speed);
		en_time = esp_timer_get_time();
		*(idx++) = en_time - st_time;
	}
	get_stadistics(st, MEASUREMENT_AMOUNT, stats[experiment::OBSERVER_STEP]);

	/* TEST IDEAL CONTROL */
	ESP_LOGI(LOG_TAG, "Ideal Control tests");
	test_controller = &ideal_control;
	test_controller->setup();
	float setpoint = 342.0f;
	idx = st;
	while ( idx < en ) {
		st_time = esp_timer_get_time();
		(void)test_controller->loop(setpoint);
		en_time = esp_timer_get_time();
		*(idx++) = en_time - st_time;
	}
	get_stadistics(st, MEASUREMENT_AMOUNT, stats[experiment::IDEAL_CONTROL_LOOP]);

	/* TEST PID CONTROL   */
	ESP_LOGI(LOG_TAG, "PID Control tests");
	test_controller = &pid_control;
	test_controller->setup();
	idx = st;
	while ( idx < en ) {
		st_time = esp_timer_get_time();
		(void)test_controller->loop(setpoint);
		en_time = esp_timer_get_time();
		*(idx++) = en_time - st_time;
	}
	get_stadistics(st, MEASUREMENT_AMOUNT, stats[experiment::PID_CONTROL_LOOP]);

	EXP_PRINT_STATS(HANDLE_PULSE,       stats);
	EXP_PRINT_STATS(FXD_TO_REPR,        stats);
	EXP_PRINT_STATS(FXD_FROM_REPR,      stats);
	EXP_PRINT_STATS(FXD_MUL,            stats);
	EXP_PRINT_STATS(OBSERVER_STEP,      stats);
	EXP_PRINT_STATS(IDEAL_CONTROL_LOOP, stats);
	EXP_PRINT_STATS(PID_CONTROL_LOOP,   stats);

	vTaskSuspend(NULL);
}

static void get_stadistics(const int64_t *const values, const size_t size, stadistics &stats) {
	const int64_t *const end = values + size;
	const int64_t *idx = values;

	int64_t max = std::numeric_limits<int64_t>::min();
	int64_t min = std::numeric_limits<int64_t>::max();

	while ( idx < end ) {
		max = std::max(max, *idx);
		min = std::min(min, *idx);
		idx++;
	}

	stats.max = int64_us_to_s( max );
	stats.min = int64_us_to_s( min );

	idx = values;
	int64_t acum = 0;
	while ( idx < end ) {
		acum += *(idx++);
	}
	stats.average = int64_us_to_s( acum / size );

	float dev_acum = 0.0f;
	idx = values;
	while ( idx < end ) {
		float deviation = int64_us_to_s( *(idx++) ) - stats.average;
		dev_acum += deviation * deviation / size;
	}
	stats.std_dev = std::sqrt(dev_acum);
}

static void print_stats(const char* title, const stadistics &stats) {
	std::printf("%s:\n", title);
	std::printf("    max    : %10.3e\n", stats.max);
	std::printf("    min    : %10.3e\n", stats.min);
	std::printf("    average: %10.3e\n", stats.average);
	std::printf("    std dev: %10.3e\n", stats.std_dev);
}

#endif