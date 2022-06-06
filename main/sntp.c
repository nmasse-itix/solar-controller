#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_sntp.h"
#include "sntp.h"
#include "common.h"

static const char *SNTP_LOGGER = "sntp";

void sntp_callback(struct timeval *tv) {
    if (sntp_get_sync_mode() == SNTP_SYNC_MODE_IMMED) {
        time_t now;
        time(&now);
        struct tm now_tm;
        localtime_r(&now, &now_tm);
        char strftime_buf[64];
        if (strftime(strftime_buf, sizeof(strftime_buf), "%c", &now_tm)) {
            ESP_LOGI(SNTP_LOGGER, "Time synchronized: %s", strftime_buf);
        }
        sntp_set_sync_mode(SNTP_SYNC_MODE_SMOOTH);
        xEventGroupSetBits(services_event_group, TIME_SYNC_BIT);
    }
}

void sntp_start() {
    ESP_LOGI(SNTP_LOGGER, "Initializing SNTP...");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_set_time_sync_notification_cb(sntp_callback);
    sntp_set_sync_mode(SNTP_SYNC_MODE_IMMED);
    sntp_init();
}
