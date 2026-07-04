/**
 * @file mock_dc_motor.hpp
 * @author ACMAX (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2026-07-02
 *
 * @copyright Copyright (c) 2026
 *
 */
#pragma once

#include "dc_plant.hpp"

namespace mock {

float get_speed     ();
float get_current   ();
float get_speed_ob  ();
float get_current_ob();

DCPlant::dc_state get_estate();
DCPlant::dc_state get_estate_ob();

void get_complete_state(
	DCPlant::dc_state &model_state,
	DCPlant::dc_state &observer_state
);

void set_load(float load_nm);

};
