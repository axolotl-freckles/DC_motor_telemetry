idf_component_register (
	SRCS
		"src/controller_task.cpp"
	INCLUDE_DIRS
		"." "include"
	REQUIRES
		"esp_timer"
		"driver"
		"globals"
		"libtask"
		"controller"
		"encoder_task"
		"telemetry_task"
	)