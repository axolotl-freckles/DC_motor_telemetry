idf_component_register (
	SRCS
		"src/telemetry_task.cpp"
	INCLUDE_DIRS
		"."
	REQUIRES
		"driver"
		"libtask"
		"esp_timer"
	)