idf_component_register (
	SRCS
		"src/telemetry_task.cpp"
	INCLUDE_DIRS
		"."
	REQUIRES
		"libtask"
		"esp_timer"
	)