idf_component_register (
	SRCS
		"src/sampler_task.cpp"
	INCLUDE_DIRS
		"."
	REQUIRES
		"esp_timer"
		"globals"
		"libtask"
		"encoder"
		"DC_model"
	)