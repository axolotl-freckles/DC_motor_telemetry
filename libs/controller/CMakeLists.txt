idf_component_register (
	SRCS
		"controller.cpp"
		"winddown.cpp"
		"windup.cpp"
		"controllers/pid_controller.cpp"
		"controllers/open_loop_controller.cpp"
		"windups/NoWindup.cpp"
		"windups/LinearWindup.cpp"
		"winddowns/NoWinddown.cpp"
		"winddowns/LinearWinddown.cpp"
	INCLUDE_DIRS "."
	REQUIRES
		"esp_timer"
		"globals"
		"time_series"
		"DC_model"
		"sampler_task"
		"telemetry_task"
	)