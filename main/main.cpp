#include <stdio.h>

#include "esp_heap_trace.h"
#include "esp_log.h"
#include "driver/ledc.h"

#include "freertos/FreeRTOS.h"

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

	vTaskDelay(pdMS_TO_TICKS(1000));
	controller_task->start();
	controller_task->wait_state(ControllerState_e::CONTROL, portMAX_DELAY);

	vTaskDelay(pdMS_TO_TICKS(12000));
	controller_task->stop();

	while (true) {
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}
