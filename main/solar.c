#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "solar.h"
#include "mqtt.h"
#include "common.h"

// 1-Wire support
#include "owb.h"
#include "owb_rmt.h"
#include "ds18b20.h"

// PWM support
#include "driver/gpio.h"
#include "driver/ledc.h"

#define SC_MAX_TEMPERATURE_SENSORS_COUNT 8
#define SC_EXPECTED_TEMPERATURE_SENSORS_COUNT 4

#define SC_PUMP_FULL_POWER 255
#define SC_PUMP_HALF_POWER 150
#define SC_PUMP_NO_POWER 0

#define SC_LEDC_CHANNEL_PANEL_PUMP LEDC_CHANNEL_0
#define SC_LEDC_CHANNEL_FLOOR_HEATING_PUMP LEDC_CHANNEL_1

static const char *SOLAR_LOGGER = "solar";
static DS18B20_Info temperature_sensors[SC_EXPECTED_TEMPERATURE_SENSORS_COUNT];
static int panel;
static int store_high;
static int store_low;
static int floor_heating;
static OneWireBus * owb;
static owb_rmt_driver_info rmt_driver_info;
static int is_heating_enabled = 0;

void solar_set_heating(int enabled) {
    is_heating_enabled = enabled;

    if (IS_READY(MQTT_CONNECTED_BIT|TIME_SYNC_BIT)) {
        mqtt_publish_data("floor_heating_enabled", (json_value){MQTT_TYPE_INT, {.i = is_heating_enabled}});
    }
}

static int map_pwm(float delta_temp) {
    float res = delta_temp * 10.2;
    
    if (res > 255) {
        return 255;
    }

    return res;
}

static void solar_process(void *pvParameters) {
    TickType_t last_wake_time = xTaskGetTickCount();
    int is_protecting_from_frost = 0;
    int is_loading = 0;
    int is_heating = 0;
    int panel_pump_pwm = 0;
    int floor_heating_pump_pwm = 0;

    int errors_count[SC_EXPECTED_TEMPERATURE_SENSORS_COUNT] = {0};
    for (;;) {
        float readings[SC_EXPECTED_TEMPERATURE_SENSORS_COUNT]; 
        DS18B20_ERROR errors[SC_EXPECTED_TEMPERATURE_SENSORS_COUNT] = { 0 };

        // Read temperatures more efficiently by starting conversions on all devices at the same time
        ds18b20_convert_all(owb);

        // In this application all devices use the same resolution,
        // so use the first device to determine the delay
        ds18b20_wait_for_conversion(&temperature_sensors[0]);


        // Read the results immediately after conversion otherwise it may fail
        // (using printf before reading may take too long)
        for (int i = 0; i < SC_EXPECTED_TEMPERATURE_SENSORS_COUNT; ++i) {
            errors[i] = ds18b20_read_temp(&temperature_sensors[i], &readings[i]);
        }

        // Check if there are errors during measure or data transmission
        int has_reading_errors = 0;
        for (int i = 0; i < SC_EXPECTED_TEMPERATURE_SENSORS_COUNT; ++i) {
            if (errors[i] != DS18B20_OK) {
                char rom_code_s[17];
                owb_string_from_rom_code(temperature_sensors[i].rom_code, rom_code_s, sizeof(rom_code_s));
                ESP_LOGW(SOLAR_LOGGER, "Error reading sensor %s.", rom_code_s);
                ++errors_count[i];
                has_reading_errors = 1;
            }
        }
        // And retry a measure if there were errors
        if (has_reading_errors) {
            ESP_LOGW(SOLAR_LOGGER, "There were errors while reading sensors. Retrying!");
            continue;
        }

        float delta_panel = readings[panel] - readings[store_low];
        float delta_floor_heating = readings[store_high] - readings[floor_heating];

        if (!is_protecting_from_frost && readings[panel] < CONFIG_SC_FROST_PROTECTION_TEMP_LOW) {
            ESP_LOGI(SOLAR_LOGGER, "Temperature is running low in the panel (%.1f), engaging frost protection!", readings[panel]);
            is_protecting_from_frost = 1;
            panel_pump_pwm = SC_PUMP_HALF_POWER;
        } else if (is_protecting_from_frost && readings[panel] > CONFIG_SC_FROST_PROTECTION_TEMP_HIGH) {
            ESP_LOGI(SOLAR_LOGGER, "Disengaging frost protection...");
            is_protecting_from_frost = 0;
            panel_pump_pwm = SC_PUMP_NO_POWER;
        } else if (!is_loading && delta_panel > CONFIG_SC_PANEL_DELTA_TEMP_HIGH) {
            ESP_LOGI(SOLAR_LOGGER, "Start loading heat from the panel to the store...");
            panel_pump_pwm = map_pwm(delta_panel);
            is_loading = 1;
        } else if (is_loading && delta_panel < CONFIG_SC_PANEL_DELTA_TEMP_LOW) {
            ESP_LOGI(SOLAR_LOGGER, "Stop loading heat from the panel to the store...");
            panel_pump_pwm = SC_PUMP_NO_POWER;
            is_loading = 0;
        } else if (is_loading) {
            // Keep adjusting the pump duty cycle according to the temperature difference
            panel_pump_pwm = map_pwm(delta_panel);
        }

        if (is_heating && !is_heating_enabled) {
            ESP_LOGI(SOLAR_LOGGER, "Stop heating the floor (as requested).");
            floor_heating_pump_pwm = SC_PUMP_NO_POWER;
            is_heating = 0;
        } else if (is_heating_enabled && !is_heating && delta_floor_heating > CONFIG_SC_FLOOR_HEATING_DELTA_TEMP_HIGH) {
            ESP_LOGI(SOLAR_LOGGER, "Start heating the floor from the store...");
            floor_heating_pump_pwm = SC_PUMP_HALF_POWER;
            is_heating = 1;
        } else if (is_heating && delta_floor_heating < CONFIG_SC_FLOOR_HEATING_DELTA_TEMP_LOW) {
            ESP_LOGI(SOLAR_LOGGER, "Stop heating the floor from the store...");
            floor_heating_pump_pwm = SC_PUMP_NO_POWER;
            is_heating = 0;
        }

        ESP_LOGI(SOLAR_LOGGER, "p:%2.1f°C, s.l:%2.1f°C, s.h:%2.1f°C, f.h:%2.1f°C, p.p: %3.0f%%, fh.p: %3.0f%%", readings[panel], readings[store_low], readings[store_high], readings[floor_heating], panel_pump_pwm / 2.55, floor_heating_pump_pwm / 2.55);

        // Set new PWM duty cycle on each pump
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_HIGH_SPEED_MODE, SC_LEDC_CHANNEL_PANEL_PUMP, panel_pump_pwm));
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_HIGH_SPEED_MODE, SC_LEDC_CHANNEL_FLOOR_HEATING_PUMP, floor_heating_pump_pwm));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_HIGH_SPEED_MODE, SC_LEDC_CHANNEL_PANEL_PUMP));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_HIGH_SPEED_MODE, SC_LEDC_CHANNEL_FLOOR_HEATING_PUMP));

        // Only publish data when connected to the MQTT broker and time is synchronized with NTP
        if (IS_READY(MQTT_CONNECTED_BIT|TIME_SYNC_BIT)) {
            mqtt_publish_data("solar_panel_temperature", (json_value){MQTT_TYPE_FLOAT, {.f = readings[panel]}});
            mqtt_publish_data("floor_temperature", (json_value){MQTT_TYPE_FLOAT, {.f = readings[floor_heating]}});
            mqtt_publish_data("store_higher_temperature", (json_value){MQTT_TYPE_FLOAT, {.f = readings[store_high]}});
            mqtt_publish_data("store_lower_temperature", (json_value){MQTT_TYPE_FLOAT, {.f = readings[store_low]}});
            mqtt_publish_data("panel_pump_duty_cycle", (json_value){MQTT_TYPE_INT, {.i = panel_pump_pwm}});
            mqtt_publish_data("floor_heating_pump_duty_cycle", (json_value){MQTT_TYPE_INT, {.i = floor_heating_pump_pwm}});
        }

        vTaskDelayUntil(&last_wake_time, (1000 * CONFIG_SC_SAMPLE_PERIOD) / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

static void solar_pwm_init() {
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer_0 = {
        .speed_mode       = LEDC_HIGH_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_8_BIT,
        .freq_hz          = CONFIG_SC_PUMP_PWM_FREQ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer_0));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel_0 = {
        .speed_mode     = LEDC_HIGH_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = CONFIG_SC_PANEL_PUMP_GPIO,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ledc_channel_config_t ledc_channel_1 = {
        .speed_mode     = LEDC_HIGH_SPEED_MODE,
        .channel        = LEDC_CHANNEL_1,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = CONFIG_SC_FLOOR_HEATING_PUMP_GPIO,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_0));
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_1));
}

static int sensor_index(char * address) {
    for (int i = 0; i < SC_EXPECTED_TEMPERATURE_SENSORS_COUNT; i++) {
        char rom_code_s[17];
        owb_string_from_rom_code(temperature_sensors[i].rom_code, rom_code_s, sizeof(rom_code_s));
        if (strcmp(address, rom_code_s) == 0) {
            return i;
        }
    }
    return -1;
}

static void solar_ds18b20_init() {
    // Stable readings require a brief period before communication
    ESP_LOGI(SOLAR_LOGGER, "Waiting before 1-wire enumeration...");
    vTaskDelay(2000.0 / portTICK_PERIOD_MS);

    // Create a 1-Wire bus, using the RMT timeslot driver
    owb = owb_rmt_initialize(&rmt_driver_info, CONFIG_SC_1WIRE_GPIO, RMT_CHANNEL_1, RMT_CHANNEL_0);
    owb_use_crc(owb, true);  // enable CRC check for ROM code

    // Find all connected temperature sensors
    ESP_LOGI(SOLAR_LOGGER, "List of all DS18B20 devices on bus:");
    OneWireBus_ROMCode device_rom_codes[SC_MAX_TEMPERATURE_SENSORS_COUNT] = {0};
    int num_devices = 0;
    OneWireBus_SearchState search_state = {0};
    bool found = false;
    owb_search_first(owb, &search_state, &found);

    while (found) {
        char rom_code_s[17];
        owb_string_from_rom_code(search_state.rom_code, rom_code_s, sizeof(rom_code_s));
        ESP_LOGI(SOLAR_LOGGER, "device %d: %s", num_devices, rom_code_s);
        device_rom_codes[num_devices] = search_state.rom_code;
        ++num_devices;
        owb_search_next(owb, &search_state, &found);
    }
    ESP_LOGI(SOLAR_LOGGER, "Found %d device%s", num_devices, num_devices == 1 ? "" : "s");

    if (num_devices != SC_EXPECTED_TEMPERATURE_SENSORS_COUNT) {
        ESP_LOGE(SOLAR_LOGGER, "Cannot find exactly %d temperature sensors, rebooting!", SC_EXPECTED_TEMPERATURE_SENSORS_COUNT);
        vTaskDelay(5000.0 / portTICK_PERIOD_MS);
        esp_restart();
    }

    // Initializes all temperature sensors
    for (int i = 0; i < num_devices; ++i)
    {
        ds18b20_init(&temperature_sensors[i], owb, device_rom_codes[i]); // associate with bus and device
        ds18b20_use_crc(&temperature_sensors[i], true); // enable CRC check on all reads
        ds18b20_set_resolution(&temperature_sensors[i], DS18B20_RESOLUTION_12_BIT); // use max. resolution
    }

    // Extract sensor addresses from NVS
    nvs_handle_t nvs;
    ESP_ERROR_CHECK(nvs_open("solar", NVS_READONLY, &nvs));
    char * panel_sensor_addr = get_nvs_string(nvs, "panel_sensor");
    assert(panel_sensor_addr != NULL);
    char * store_higher_sensor_addr = get_nvs_string(nvs, "store_h_sensor");
    assert(store_higher_sensor_addr != NULL);
    char * store_lower_sensor_addr = get_nvs_string(nvs, "store_l_sensor");
    assert(store_lower_sensor_addr != NULL);
    char * floor_heating_sensor_addr = get_nvs_string(nvs, "fl_ht_sensor");
    assert(floor_heating_sensor_addr != NULL);

    if (panel_sensor_addr == NULL || store_higher_sensor_addr == NULL || store_lower_sensor_addr == NULL || floor_heating_sensor_addr == NULL) {
        ESP_LOGE(SOLAR_LOGGER, "Cannot find one of the temperature sensor addresses, check your configuration!");
        vTaskDelay(5000.0 / portTICK_PERIOD_MS);
        esp_restart();
    }

    // Match each sensor with its function
    panel = sensor_index(panel_sensor_addr);
    assert(panel != -1);
    store_high = sensor_index(store_higher_sensor_addr);
    assert(store_high != -1);
    store_low = sensor_index(store_lower_sensor_addr);
    assert(store_low != -1);
    floor_heating = sensor_index(floor_heating_sensor_addr);
    assert(floor_heating != -1);

    if (panel == -1 || store_high == -1 || store_low == -1 || floor_heating == -1) {
        ESP_LOGE(SOLAR_LOGGER, "Cannot find one of the temperature sensors, check your configuration!");
        vTaskDelay(5000.0 / portTICK_PERIOD_MS);
        esp_restart();
    }
}

void solar_init() {
    solar_pwm_init();
    solar_ds18b20_init();

    // Create a task to handle solar monitoring
    BaseType_t xReturned;
    xReturned = xTaskCreate(solar_process,
                            "solar_process",
                            CONFIG_SC_STACK_SIZE,  /* Stack size in words, not bytes. */
                            NULL,           /* Parameter passed into the task. */
                            tskIDLE_PRIORITY + 12,
                            NULL);
    if (xReturned != pdPASS) {
        ESP_LOGE(SOLAR_LOGGER, "xTaskCreate('solar_process'): %d", xReturned);
        abort();
    }
}
