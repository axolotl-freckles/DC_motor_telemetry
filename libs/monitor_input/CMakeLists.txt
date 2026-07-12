idf_component_register (
	SRCS
		"src/monitor_input.cpp"
	INCLUDE_DIRS "."
	REQUIRES
		"esp_timer"
		"driver"
		"libtask"
		"controller_task"
		"sampler_task"
)
