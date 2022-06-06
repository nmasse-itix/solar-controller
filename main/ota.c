#include <string.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "esp_https_ota.h"
#include "esp_ota_ops.h"
#include "ota.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "common.h"

static const char *OTA_LOGGER = "ota_update";

// Embedded via component.mk
extern const uint8_t cacert_pem_start[]   asm("_binary_cacert_pem_start");
extern const uint8_t cacert_pem_end[]   asm("_binary_cacert_pem_end");

esp_err_t do_firmware_upgrade(char* firmware_url) {
    esp_http_client_config_t config = {
        .url = firmware_url,
        .cert_pem = (char *)cacert_pem_start,
    };

    esp_https_ota_config_t ota_config = {
        // Can be enabled in an upcoming version of the Espressif SDK
        // to save some memory.
        // Requires CONFIG_MBEDTLS_SSL_IN_CONTENT_LEN=4096 in sdkconfig.
        //
        //.partial_http_download = true,
        //.max_http_request_size = 4096,
        .http_config = &config,
    };

    esp_https_ota_handle_t https_ota_handle = NULL;
    esp_err_t err = esp_https_ota_begin(&ota_config, &https_ota_handle);
    if (https_ota_handle == NULL) {
        return ESP_FAIL;
    }

    while (1) {
        err = esp_https_ota_perform(https_ota_handle);
        if (err != ESP_ERR_HTTPS_OTA_IN_PROGRESS) {
            break;
        }
    }

    if (err != ESP_OK) {
        esp_https_ota_abort(https_ota_handle);
        return err;
    }

    esp_err_t ota_finish_err = esp_https_ota_finish(https_ota_handle);
    if (ota_finish_err != ESP_OK) {
        return ota_finish_err;
    }

    esp_restart(); // this function never returns

    return ESP_OK;
}

void trigger_ota_update(char* version) {
    const esp_app_desc_t* current = esp_ota_get_app_description();
    ESP_LOGD(OTA_LOGGER, "Currently running %s version %s", current->project_name, current->version);

    if (strcmp(version, current->version) == 0) {
        // already to latest version
        return;
    }

    nvs_handle_t nvs;
    esp_err_t err = nvs_open("ota", NVS_READONLY, &nvs);
    if (err != ESP_OK) {
        ESP_LOGE(OTA_LOGGER, "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        return;
    }
    char* update_url_pattern = get_nvs_string(nvs, "update_url");
    nvs_close(nvs);
    if (update_url_pattern == NULL) {
        return;
    }

    const size_t buffer_size = 256;
    char update_url[buffer_size];

    // Format the update URL
    if (!snprintf(update_url, buffer_size, update_url_pattern, version)) {
        ESP_LOGD(OTA_LOGGER, "trigger_ota_update: snprintf failed!");
        free(update_url_pattern);
        return;
    }

    esp_err_t ret = do_firmware_upgrade(update_url);
    ESP_LOGW(OTA_LOGGER, "do_firmware_upgrade failed with error %d", ret);
}
