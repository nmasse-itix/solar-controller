#ifndef __COMMON_H__
#define __COMMON_H__

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"
#include "nvs.h"

#define WIFI_CONNECTED_BIT BIT0
#define MQTT_CONNECTED_BIT BIT1
#define TIME_SYNC_BIT BIT2

extern volatile EventGroupHandle_t services_event_group;
char* get_nvs_string(nvs_handle_t nvs, char* key);

#define WAIT_FOR(flags) while ((xEventGroupWaitBits(services_event_group, flags, pdFALSE, pdTRUE, portMAX_DELAY) & (flags)) != (flags)) {}
#define IS_READY(flags) ((xEventGroupGetBits(services_event_group) & (flags)) == (flags))

#endif
