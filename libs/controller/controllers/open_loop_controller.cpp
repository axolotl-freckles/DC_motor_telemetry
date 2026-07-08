/**
 * @file open_loop_controller.cpp
 * @author ACMAX (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2026-06-22
 *
 * @copyright Copyright (c) 2026
 *
 */
#include "open_loop_controller.hpp"

#include <stdio.h>

#include "esp_timer.h"

#include "telemetry_task.hpp"
#include "num_calculus.hpp"

static void interpolate_simulation(
	uint64_t                   &last_interpolation_us,
	DCPlant::EulerDCMotorModel &model,
	DCPlant::DCMotorObserver   &observer,
	float    const              amature_voltage,
	float    const              load,
	Integrator *const integrator_w = nullptr,
	Integrator *const integrator_I = nullptr,
	Derivator  *const derivator_w  = nullptr,
	Derivator  *const derivator_I  = nullptr
) {
	const uint64_t now_us          = esp_timer_get_time();
	const uint64_t elapsed_time_ms = (now_us-last_interpolation_us)/1000L;
	const uint32_t missing_step_n  = (uint32_t)elapsed_time_ms/MODEL_SIM_TIME_ms;

	for (uint32_t i=0; i<missing_step_n; i++) {
		if (derivator_w)  { derivator_w->operator()(model.state().w_rad_s); }
		if (derivator_I)  { derivator_I->operator()(model.state().I_amp);   }
		if (integrator_w) { integrator_w->operator()(model.state().w_rad_s); }
		if (integrator_I) { integrator_I->operator()(model.state().I_amp);   }

		observer.step(amature_voltage, model.state());
		model   .step(amature_voltage, load);
	}
	last_interpolation_us = now_us;
}

OpenLoop::OpenLoop (QueueHandle_t voltage_q) :
	_voltage_q (voltage_q),
	_estimator (SAMPLE_PARAMS, MODEL_SIM_TIME_s),
	_observer  (SAMPLE_PARAMS, SAMPLE_OBS_PRMS, MODEL_SIM_TIME_s)
{}

void OpenLoop::setup () {
	_estimator.reset();
	_observer .reset();
	_start_time_us = esp_timer_get_time();
}

void OpenLoop::loop () {
	static    uint8_t  n_ticks        = 1;
	constexpr float    SIMULATED_LOAD = 1.0f;
	const     uint64_t now            = esp_timer_get_time();
	//_voltage_setpoint = 30.0f;

	// interpolate_simulation (
	// 	_start_time_us,
	// 	_estimator,
	// 	_observer,
	// 	_voltage_setpoint,
	// 	SIMULATED_LOAD
	// );


	if (n_ticks >= 4) {
		task::telemetry::telemetry_data_t package = {
			.timestamp      = now*1e-6f,
			.setpoint       = _voltage_setpoint,
			.set_voltage    = _voltage_setpoint,
			// .w_rad_s        = _estimator.state().w_rad_s,
			.w_rad_s        = read_speed_rad_s(),
			//.I_amp          = _estimator.state().I_amp,
			.I_amp          = read_current(),
			.estimated_load = _observer.estimated_load()
		};
		xQueueSend(
			task::telemetry::TelemetryTask::get_instance().data_queue(),
			&package,
			0
		);
		n_ticks = 0;
	}
	n_ticks++;

	(void)xQueuePeek(_voltage_q, &_voltage_setpoint, portMAX_DELAY);
	set_voltage(_voltage_setpoint);
}