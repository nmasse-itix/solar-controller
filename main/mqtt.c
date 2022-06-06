#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "esp_tls.h"
#include "esp_ota_ops.h"
#include <sys/param.h>
#include <time.h>
#include "cJSON.h"
#include "common.h"
#include "ota.h"
#include "mqtt.h"
#include "solar.h"

static esp_mqtt_client_config_t mqtt_cfg;
static esp_mqtt_client_handle_t client;

static const char *MQTT_LOGGER = "mqtt";

#define MQTT_QOS_0 0
#define MQTT_QOS_1 1
#define MQTT_QOS_2 2

#define MQTT_NO_RETAIN 0
#define MQTT_RETAIN 1

#define JSON_BUFFER_SIZE 128
#define MQTT_TOPIC_BUFFER_SIZE 128

#define SYSTEM_COMMAND_UPDATE "firmware-update"
#define SYSTEM_COMMAND_REBOOT "reboot"
#define SYSTEM_COMMAND_SET_PARAM "set-parameter"
#define PARAMETER_HEATING_ENABLED "heating_enabled"

void mqtt_publish_data(char* key, json_value jv) {
    char topic[MQTT_TOPIC_BUFFER_SIZE];
    char payload[JSON_BUFFER_SIZE];
    time_t now;
    int retain = MQTT_RETAIN;
    int qos = MQTT_QOS_1;

    // Format the MQTT topic
    if (!snprintf(topic, MQTT_TOPIC_BUFFER_SIZE, CONFIG_MQTT_SOLAR_VALUE_TOPIC, key)) {
        ESP_LOGD(MQTT_LOGGER, "mqtt_publish_data: snprintf failed!");
        return;
    }

    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        ESP_LOGD(MQTT_LOGGER, "mqtt_publish_data: cJSON_CreateObject failed!");
        return;
    }

    // Add the value
    if (jv.type == MQTT_TYPE_STRING) {
        cJSON_AddStringToObject(root, "val", (char*)jv.value.str);
    } else if (jv.type == MQTT_TYPE_FLOAT) {
        float f = jv.value.f;
        cJSON_AddNumberToObject(root, "val", (double)f);
    } else if (jv.type == MQTT_TYPE_INT) {
        int i = jv.value.i;
        cJSON_AddNumberToObject(root, "val", (double)i);
    }

    // Add a timestamp
    time(&now);
    cJSON_AddNumberToObject(root, "ts", (double)now);

    if (!cJSON_PrintPreallocated(root, payload, JSON_BUFFER_SIZE, 0)) {
        ESP_LOGD(MQTT_LOGGER, "mqtt_publish_data: cJSON_PrintPreallocated failed!");
        cJSON_Delete(root);
        return;
    }
    cJSON_Delete(root);

    if (esp_mqtt_client_publish(client, topic, payload, 0, qos, retain) == -1) {
        ESP_LOGD(MQTT_LOGGER, "MQTT Message discarded!");
    }
}

esp_err_t mqtt_process_system_command(char* data, int data_len) {
    esp_err_t status = ESP_OK;
    cJSON *json = cJSON_ParseWithLength(data, data_len);
    if (json == NULL) {
        ESP_LOGI(MQTT_LOGGER, "Error parsing MQTT system command as JSON");
        status = ESP_FAIL;
        goto end;
    }

    cJSON* command = cJSON_GetObjectItemCaseSensitive(json, "command");
    if (!cJSON_IsString(command) || (command->valuestring == NULL)) {
        ESP_LOGD(MQTT_LOGGER, "Expected a command name!");
        status = ESP_FAIL;
        goto end;
    }

    if (strcmp(command->valuestring, SYSTEM_COMMAND_REBOOT) == 0) {
        ESP_LOGE(MQTT_LOGGER, "Received a reboot command. Rebooting now!");
        esp_restart();
        goto end;
    }

    if (strcmp(command->valuestring, SYSTEM_COMMAND_UPDATE) == 0) {
        cJSON* args = cJSON_GetObjectItemCaseSensitive(json, "args");
        if (!cJSON_IsObject(args)) {
            ESP_LOGD(MQTT_LOGGER, "Expected a command argument!");
            status = ESP_FAIL;
            goto end;
        }

        cJSON* version = cJSON_GetObjectItemCaseSensitive(args, "version");
        if (!cJSON_IsString(version) || (version->valuestring == NULL)) {
            ESP_LOGD(MQTT_LOGGER, "Expected a version numer!");
            status = ESP_FAIL;
            goto end;
        }

        trigger_ota_update(version->valuestring);
        goto end;
    } else if (strcmp(command->valuestring, SYSTEM_COMMAND_SET_PARAM) == 0) {
        cJSON* args = cJSON_GetObjectItemCaseSensitive(json, "args");
        if (!cJSON_IsObject(args)) {
            ESP_LOGD(MQTT_LOGGER, "Expected a command argument!");
            status = ESP_FAIL;
            goto end;
        }

        cJSON* name = cJSON_GetObjectItemCaseSensitive(args, "name");
        if (!cJSON_IsString(name) || (name->valuestring == NULL)) {
            ESP_LOGD(MQTT_LOGGER, "Expected a parameter name!");
            status = ESP_FAIL;
            goto end;
        }

        char * param_name = name->valuestring;
        if (strcmp(param_name, PARAMETER_HEATING_ENABLED) == 0) {
            cJSON* value = cJSON_GetObjectItemCaseSensitive(args, "value");
            if (!cJSON_IsNumber(value)) {
                ESP_LOGD(MQTT_LOGGER, "Expected a parameter value with type 'number'!");
                status = ESP_FAIL;
                goto end;
            }
            int heating_enabled = value->valuedouble;
            solar_set_heating(heating_enabled);
        }

        goto end;
    }

    ESP_LOGW(MQTT_LOGGER, "Unknown system command %s!", command->valuestring);
    status = ESP_FAIL;

    end:
    cJSON_Delete(json);
    return status;
}

esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event) {
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(MQTT_LOGGER, "MQTT_EVENT_CONNECTED");
            xEventGroupSetBits(services_event_group, MQTT_CONNECTED_BIT);
            if (esp_mqtt_client_publish(client, CONFIG_MQTT_LWT_TOPIC, "1", 0, MQTT_QOS_0, MQTT_RETAIN) == -1) {
                ESP_LOGD(MQTT_LOGGER, "MQTT Message discarded!");
            }
            if (esp_mqtt_client_subscribe(client, CONFIG_MQTT_COMMAND_TOPIC, MQTT_QOS_1) == -1) {
                ESP_LOGD(MQTT_LOGGER, "Could not subscribe to " CONFIG_MQTT_COMMAND_TOPIC " MQTT topic");
            }
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(MQTT_LOGGER, "MQTT_EVENT_DISCONNECTED");
            xEventGroupClearBits(services_event_group, MQTT_CONNECTED_BIT);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGD(MQTT_LOGGER, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(MQTT_LOGGER, "MQTT_EVENT_ERROR");
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                ESP_LOGI(MQTT_LOGGER, "Last error code reported from esp-tls: 0x%x", event->error_handle->esp_tls_last_esp_err);
                ESP_LOGI(MQTT_LOGGER, "Last tls stack error number: 0x%x", event->error_handle->esp_tls_stack_err);
                ESP_LOGI(MQTT_LOGGER, "Last captured errno : %d (%s)",  event->error_handle->esp_transport_sock_errno,
                                                                strerror(event->error_handle->esp_transport_sock_errno));
            } else if (event->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
                ESP_LOGI(MQTT_LOGGER, "Connection refused error: 0x%x", event->error_handle->connect_return_code);
            } else {
                ESP_LOGW(MQTT_LOGGER, "Unknown error type: 0x%x", event->error_handle->error_type);
            }
            break;
        case MQTT_EVENT_DATA:
            if (strncmp(event->topic, CONFIG_MQTT_COMMAND_TOPIC, event->topic_len) == 0) {
                ESP_LOGD(MQTT_LOGGER, "Received an MQTT system command!");
                mqtt_process_system_command(event->data, event->data_len);
            }
            break;
        case MQTT_EVENT_SUBSCRIBED:
            // Expected event. Nothing to do.
            break;
        default:
            ESP_LOGD(MQTT_LOGGER, "Other event id:%d", event->event_id);
            break;
    }
    return ESP_OK;
}

void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGD(MQTT_LOGGER, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    mqtt_event_handler_cb(event_data);
}

void mqtt_init(void) {
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("mqtt", NVS_READONLY, &nvs);
    if (err != ESP_OK) {
        ESP_LOGE(MQTT_LOGGER, "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        return;
    }

    memset(&mqtt_cfg, 0, sizeof(mqtt_cfg));
    mqtt_cfg.uri = get_nvs_string(nvs, "url");
    mqtt_cfg.use_global_ca_store = true;
    mqtt_cfg.username = get_nvs_string(nvs, "username");
    mqtt_cfg.password = get_nvs_string(nvs, "password");
    mqtt_cfg.lwt_topic = CONFIG_MQTT_LWT_TOPIC;
    mqtt_cfg.lwt_msg = "0";
    mqtt_cfg.lwt_qos = MQTT_QOS_0;
    mqtt_cfg.lwt_retain = MQTT_RETAIN;

    nvs_close(nvs);

    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
}
