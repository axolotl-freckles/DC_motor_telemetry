#include <stdio.h>

#include "esp_log.h"

#include "freertos/FreeRTOS.h"

#include "controller.hpp"
#include "controllers/pid_controller.hpp"
#include "filters.hpp"

#include "tasks.hpp"
#include "controller_task.hpp"
#include "encoder_task.hpp"

using task::controller::ControllerTask;
using task::encoder::EncoderTask;

extern "C" void app_main(void)
{
	ControllerTask controller_task = ControllerTask::get_instance();
	EncoderTask    encoder_task    = EncoderTask::get_instance();

	QueueHandle_t setpoint_qh = xQueueCreate(1, sizeof(float));
	QueueHandle_t speed_qh    = xQueueCreate(1, sizeof(float));
	QueueHandle_t cpoint_qh   = xQueueCreate(1, sizeof(float));
	ControllerTask::config_params controller_config = {
		.setpoint_qh       = setpoint_qh,
		.speed_qh          = speed_qh,
		.control_signal_qh = cpoint_qh
	};
	controller_task.set_params(controller_config);

	vTaskDelay(pdMS_TO_TICKS(3000));
	controller_task.start();

	vTaskDelay(pdMS_TO_TICKS(12000));
	controller_task.stop();

	while (true) {
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}
