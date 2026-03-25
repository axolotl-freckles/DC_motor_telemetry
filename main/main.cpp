#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "console_advanced.hpp"

extern "C" void app_main(void)
{
	// Initialize the advanced console
	console_advanced_init();
	
	// Start the console main loop
	console_advanced_start();
}
