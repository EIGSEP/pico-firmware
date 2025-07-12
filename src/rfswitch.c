#include "rfswitch.h"
#include "pico/stdlib.h"
#include "cJSON.h"
#include <stdlib.h>

static RFSwitch rfswitch;

void rfswitch_init(uint8_t app_id) {
    rfswitch.sw_state = 0;
    rfswitch.pins[0] = RFSWITCH0_PIN;
    rfswitch.pins[1] = RFSWITCH1_PIN;
    rfswitch.pins[2] = RFSWITCH2_PIN;
    rfswitch.pins[3] = RFSWITCH3_PIN;
    rfswitch.pins[4] = RFSWITCH4_PIN;
    rfswitch.pins[5] = RFSWITCH5_PIN;
    rfswitch.pins[6] = RFSWITCH6_PIN;
    rfswitch.pins[7] = RFSWITCH7_PIN;
    for (int i = 0; i < 8; i++) {
        gpio_init(rfswitch.pins[i]);
        gpio_set_dir(rfswitch.pins[i], GPIO_OUT);
    }
}


void rfswitch_server(uint8_t app_id, const char *json_str) {
    cJSON *root = cJSON_Parse(json_str);
    if (root == NULL) return;
    cJSON *sw_state_json = cJSON_GetObjectItem(root, "sw_state");
    if (sw_state_json) {
        rfswitch.sw_state = sw_state_json ? sw_state_json->valueint : rfswitch.sw_state;
    }
    cJSON_Delete(root);
}


void rfswitch_status(uint8_t app_id) {
	send_json(4,
        KV_STR, "sensor_name", "rfswitch",
        KV_STR, "status", "update",
        KV_INT, "app_id", app_id,
        KV_INT, "sw_state", rfswitch.sw_state
    );
}

void rfswitch_op(uint8_t app_id) {
    for (int i = 0; i < 8; i++) {
        gpio_put(rfswitch.pins[i], (rfswitch.sw_state >> i) & 0x1);
    }
}
