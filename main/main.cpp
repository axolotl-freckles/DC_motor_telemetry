#include <stdio.h>

#include "esp_log.h"
#include "driver/ledc.h"
#include "driver/uart.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "controller_task.hpp"
#include "apply_task.hpp"
#include "sampler_task.hpp"
#include "monitor_input.hpp"

using task::controller::ControllerTask;
using task::controller::ControllerState_e;
using task::sampler::SamplerTask;
using task::apply::ApplyTask;
using task::monitor_in::MonitorInput;

extern "C" void app_main(void)
{
	ControllerTask &controller_task = ControllerTask::get_instance();
	SamplerTask    &sampler_task    = SamplerTask::get_instance();
	ApplyTask      &apply_task      = ApplyTask::get_instance();
	//MonitorInput   &monitor_task    = MonitorInput::get_instance();
	ledc_fade_func_install(0);

	QueueHandle_t setpoint_qh = xQueueCreate(1, sizeof(float));
	QueueHandle_t cpoint_qh   = xQueueCreate(1, sizeof(float));

	task::controller::ControllerTask::config_params controller_config = {
		.setpoint_qh       = setpoint_qh,
		.control_signal_qh = cpoint_qh
	};
	SamplerTask::config_params sampler_config = { };
	ApplyTask::config_params apply_config = {
		.voltage_queue_h = cpoint_qh
	};

	controller_task.set_params(controller_config);
	sampler_task.set_params(sampler_config);
	apply_task.set_params(apply_config);
	//monitor_task.set_setpoint_qh(setpoint_qh);

	controller_task.start();
	//monitor_task.start();
	vTaskDelay(pdMS_TO_TICKS(1000));
	float setpoint_init_rad_s = 20.0f;
	xQueueOverwrite(setpoint_qh, &setpoint_init_rad_s);

	while (true) {
		vTaskDelay(pdMS_TO_TICKS(portMAX_DELAY));
	}

	// Keep task alive after test to preserve logs and state.
}

