idf_component_register (
	SRCS
		"src/apply_task.cpp"
	INCLUDE_DIRS "."
	REQUIRES
		"esp_timer"
		"driver"
		"globals"
		"libtask"
	)

