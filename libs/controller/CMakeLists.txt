idf_component_register (
	SRCS
		"controller.cpp"
		"winddown.cpp"
		"windup.cpp"
		"controllers/pid_controller.cpp"
		"controllers/open_loop_controller.cpp"
	INCLUDE_DIRS "."
	REQUIRES
		"time_series"
		"DC_model"
	)