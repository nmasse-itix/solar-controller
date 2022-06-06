#include "common.h"
volatile EventGroupHandle_t services_event_group;

char* get_nvs_string(nvs_handle_t nvs, char* key) {
    size_t required_size;
    esp_err_t err = nvs_get_str(nvs, key, NULL, &required_size);
    if (err != ESP_OK) {
        return NULL;
    }
    char* value = malloc(required_size);
    if (!value) {
        return NULL;
    }
    err = nvs_get_str(nvs, key, value, &required_size);
    if (err != ESP_OK) {
        free(value);
        return NULL;
    }
    return value;
}
