idf_component_register (
	SRCS
		"src/controller.cpp"
	INCLUDE_DIRS
		"." "include"
	REQUIRES
		"esp_timer"
		"libtask"
		"controller"
	)