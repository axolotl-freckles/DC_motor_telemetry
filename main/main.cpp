#include <stdio.h>
#include <string.h>
#include <cmath>

#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "driver/ledc.h"
#include "driver/uart.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "controller_task.hpp"
#include "apply_task.hpp"
#include "sampler_task.hpp"
#include "telemetry_task.hpp"

using task::controller::ControllerTask;
using task::controller::ControllerState_e;
using task::sampler::SamplerTask;
using task::apply::ApplyTask;

static const char *TAG = "main";
static EventGroupHandle_t s_wifi_event_group = nullptr;

static constexpr EventBits_t WIFI_CONNECTED_BIT = BIT0;
static constexpr EventBits_t WIFI_FAIL_BIT      = BIT1;
static constexpr int WIFI_MAX_RETRY = 10;
static int s_wifi_retry = 0;

static constexpr float VOLTAGE_BATTERY = 25.0f;
static constexpr float DUTY_MIN_PERCENT = 5.0f;
static constexpr float DUTY_MAX_PERCENT = 95.0f;

// Update these credentials for your test network.
static constexpr const char *WIFI_SSID = "CHEMA";
static constexpr const char *WIFI_PASS = "12345678";

static float clampf(float x, float lo, float hi) {
	if (x < lo) {
		return lo;
	}
	if (x > hi) {
		return hi;
	}
	return x;
}

static void wifi_event_handler(
	void *arg,
	esp_event_base_t event_base,
	int32_t event_id,
	void *event_data
) {
	(void)arg;
	(void)event_data;

	if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
		esp_wifi_connect();
	}
	else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
		if (s_wifi_retry < WIFI_MAX_RETRY) {
			esp_wifi_connect();
			s_wifi_retry++;
			ESP_LOGW(TAG, "WiFi reconnect %d/%d", s_wifi_retry, WIFI_MAX_RETRY);
		}
		else {
			xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
		}
	}
	else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
		s_wifi_retry = 0;
		xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
	}
}

static esp_err_t wifi_init_sta(void) {
	s_wifi_event_group = xEventGroupCreate();
	if (s_wifi_event_group == nullptr) {
		return ESP_ERR_NO_MEM;
	}

	if (esp_netif_create_default_wifi_sta() == nullptr) {
		ESP_LOGE(TAG, "esp_netif_create_default_wifi_sta failed");
		return ESP_FAIL;
	}

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));

	esp_event_handler_instance_t wifi_any_id;
	esp_event_handler_instance_t wifi_got_ip;
	ESP_ERROR_CHECK(esp_event_handler_instance_register(
		WIFI_EVENT,
		ESP_EVENT_ANY_ID,
		&wifi_event_handler,
		nullptr,
		&wifi_any_id
	));
	ESP_ERROR_CHECK(esp_event_handler_instance_register(
		IP_EVENT,
		IP_EVENT_STA_GOT_IP,
		&wifi_event_handler,
		nullptr,
		&wifi_got_ip
	));

	wifi_config_t wifi_config = {};
	memcpy(wifi_config.sta.ssid, WIFI_SSID, strlen(WIFI_SSID));
	memcpy(wifi_config.sta.password, WIFI_PASS, strlen(WIFI_PASS));
	wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
	wifi_config.sta.pmf_cfg.capable = true;
	wifi_config.sta.pmf_cfg.required = false;

	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
	ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
	ESP_ERROR_CHECK(esp_wifi_start());

	EventBits_t bits = xEventGroupWaitBits(
		s_wifi_event_group,
		WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
		pdFALSE,
		pdFALSE,
		pdMS_TO_TICKS(20000)
	);

	if (bits & WIFI_CONNECTED_BIT) {
		ESP_LOGI(TAG, "WiFi connected to SSID: %s", WIFI_SSID);
		return ESP_OK;
	}

	ESP_LOGE(TAG, "WiFi connection failed (SSID: %s)", WIFI_SSID);
	return ESP_FAIL;
}

extern "C" void app_main(void)
{
	esp_err_t nvs_ret = nvs_flash_init();
	if ((nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES) || (nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND)) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		nvs_ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(nvs_ret);
	ESP_ERROR_CHECK(esp_netif_init());
	ESP_ERROR_CHECK(esp_event_loop_create_default());
	if (wifi_init_sta() != ESP_OK) {
		ESP_LOGW(TAG, "Continuing without WiFi; WebSocket telemetry may be unavailable");
	}

	ControllerTask &controller_task = ControllerTask::get_instance();
	SamplerTask    &sampler_task    = SamplerTask::get_instance();
	ApplyTask      &apply_task      = ApplyTask::get_instance();
	task::telemetry::TelemetryTask &telemetry_task = task::telemetry::TelemetryTask::get_instance();
	//MonitorInput   &monitor_task    = MonitorInput::get_instance();
	ledc_fade_func_install(0);

	QueueHandle_t setpoint_qh = xQueueCreate(1, sizeof(float));
	QueueHandle_t cpoint_qh   = xQueueCreate(1, sizeof(float));

	task::controller::ControllerTask::config_params controller_config = {
		.setpoint_qh       = setpoint_qh,
		.control_signal_qh = cpoint_qh
	};
	SamplerTask::config_params sampler_config = { };
	ApplyTask::config_params apply_config = {
		.voltage_queue_h = cpoint_qh
	};

	controller_task.set_params(controller_config);
	sampler_task.set_params(sampler_config);
	apply_task.set_params(apply_config);
	QueueHandle_t ws_setpoint_qh = telemetry_task.ws_setpoint_queue();
	//monitor_task.set_setpoint_qh(setpoint_qh);

	controller_task.start();
	//monitor_task.start();
	vTaskDelay(pdMS_TO_TICKS(1000));
	float duty_percent = 20.0f;
	float duty = duty_percent * 0.01f;
	float control_voltage = VOLTAGE_BATTERY * duty / (1.0f - duty);
	xQueueOverwrite(setpoint_qh, &control_voltage);

	while (true) {
		float requested_duty_percent = 0.0f;
		if ((ws_setpoint_qh != nullptr) && (pdTRUE == xQueueReceive(ws_setpoint_qh, &requested_duty_percent, 0))) {
			const float clamped_percent = clampf(requested_duty_percent, DUTY_MIN_PERCENT, DUTY_MAX_PERCENT);
			duty_percent = clamped_percent;
			duty = duty_percent * 0.01f;
			control_voltage = VOLTAGE_BATTERY * duty / (1.0f - duty);
			xQueueOverwrite(setpoint_qh, &control_voltage);

			if (fabsf(clamped_percent - requested_duty_percent) > 1e-3f) {
				ESP_LOGW(TAG, "WS setpoint %.2f%% out of range, clamped to %.2f%%", requested_duty_percent, clamped_percent);
			}
			else {
				ESP_LOGI(TAG, "WS duty setpoint applied: %.2f%%", duty_percent);
			}
		}

		vTaskDelay(pdMS_TO_TICKS(50));
	}

	// Keep task alive after test to preserve logs and state.
}

