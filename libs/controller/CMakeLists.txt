idf_component_register (
	SRCS
		"controller.cpp"
		"winddown.cpp"
		"windup.cpp"
		"controllers/pid_controller.cpp"
	INCLUDE_DIRS "."
	REQUIRES "time_series"
	)