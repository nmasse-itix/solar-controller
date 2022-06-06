#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_tls.h"
#include "esp_crt_bundle.h"
#include "wifi.h"
#include "mqtt.h"
#include "solar.h"
#include "sntp.h"
#include "common.h"
#include "esp_ota_ops.h"
#include "ota.h"

// Embedded via component.mk
extern const uint8_t cacert_pem_start[]   asm("_binary_cacert_pem_start");
extern const uint8_t cacert_pem_end[]   asm("_binary_cacert_pem_end");

void app_main(void) {
    const esp_app_desc_t* current = esp_ota_get_app_description();
    ESP_LOGI("main", "Currently running %s version %s", current->project_name, current->version);

    // NVS is used to store wifi credentials. So, we need to initialize it first.
    ESP_ERROR_CHECK(nvs_flash_init());

    // Inject the Let's Encrypt Root CA certificate in the global CA store
    ESP_ERROR_CHECK(esp_tls_init_global_ca_store());
    ESP_ERROR_CHECK(esp_tls_set_global_ca_store((const unsigned char *) cacert_pem_start, cacert_pem_end - cacert_pem_start));

    // Create an event group to keep track of service readiness
    services_event_group = xEventGroupCreate();

    // Start the solar controller early because we need it working even in case
    // of missing wifi network.
    solar_init();

    // Then connect to Wifi, MQTT and NTP
    wifi_init_sta();
    WAIT_FOR(WIFI_CONNECTED_BIT);
    mqtt_init();
    sntp_start();
    WAIT_FOR(MQTT_CONNECTED_BIT|TIME_SYNC_BIT);
}
