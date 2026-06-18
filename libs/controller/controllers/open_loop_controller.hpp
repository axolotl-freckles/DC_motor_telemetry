/**
 * @file open_loop_controller.hpp
 * @author ACMAX (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2026-06-17
 *
 * @copyright Copyright (c) 2026
 *
 */
#pragma once

#include "../controller.hpp"

#include "freertos/FreeRTOS.h"

class OpenLoop : public Controller {
public:
	OpenLoop (QueueHandle_t voltage_q);

	void setup() override;
	void loop()  override;
private:
	QueueHandle_t _voltage_q;
};
