#ifndef __MQTT_H__
#define __MQTT_H__

#define MQTT_TYPE_UNDEFINED 0
#define MQTT_TYPE_STRING    1
#define MQTT_TYPE_INT       2
#define MQTT_TYPE_FLOAT     3

typedef struct {
    int type;
    union {
        char* str;
        float f;
        int i;
    } value;
} json_value;


void mqtt_init();
void mqtt_publish_data(char* key, json_value value);

#endif
