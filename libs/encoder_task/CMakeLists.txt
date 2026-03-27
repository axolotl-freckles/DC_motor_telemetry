idf_component_register (
	SRCS
		"src/encoder.cpp"
	INCLUDE_DIRS
		"." "include"
	REQUIRES
		"esp_timer"
		"libtask"
	)