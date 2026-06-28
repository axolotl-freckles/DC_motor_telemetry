idf_component_register (
	SRCS
		"src/controller_task.cpp"
	INCLUDE_DIRS
		"." "include"
	REQUIRES
		"esp_timer"
		"globals"
		"libtask"
		"controller"
		"encoder_task"
	)