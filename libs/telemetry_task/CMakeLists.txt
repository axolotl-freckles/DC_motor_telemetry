idf_component_register (
	SRCS
		"src/telemetry_task.cpp"
	INCLUDE_DIRS
		"."
	REQUIRES
		"driver"
		"libtask"
		"esp_timer"
		"log"
		"esp_netif"
		"esp_event"
		"esp_websocket_client"
	)