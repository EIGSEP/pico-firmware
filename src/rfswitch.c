#include "rfswitch.h"
#include "pico/stdlib.h"
#include "cJSON.h"
#include <stdlib.h>

static RFSwitch rfswitch;

void rfswitch_init(uint8_t app_id) {
    rfswitch.commanded_state = 0;
    rfswitch.reported_state = 0;
    // Boot starts a transition: the physical switch position is not
    // knowable until the settle timer elapses, even though GPIOs drive
    // to 0 immediately.
    rfswitch.in_transition = true;
    rfswitch.transition_end = make_timeout_time_ms(SWITCH_SETTLE_MS);
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
    if (!root || !cJSON_IsObject(root)) {
        cJSON_Delete(root);
        return;
    }
    cJSON *sw_state_json = cJSON_GetObjectItem(root, "sw_state");
    if (sw_state_json) {
        int new_state = sw_state_json->valueint;
        // Only re-enter a transition when the commanded state actually
        // changes; repeated commands at the current state are no-ops
        // so we don't smear a settled position into UNKNOWN.
        if (new_state != rfswitch.commanded_state) {
            rfswitch.commanded_state = new_state;
            rfswitch.transition_end = make_timeout_time_ms(SWITCH_SETTLE_MS);
            rfswitch.in_transition = true;
        }
    }
    cJSON_Delete(root);
}


void rfswitch_status(uint8_t app_id) {
    int reported = rfswitch.in_transition
        ? SW_STATE_UNKNOWN
        : rfswitch.reported_state;
    send_json(4,
        KV_STR, "sensor_name", "rfswitch",
        KV_STR, "status", "update",
        KV_INT, "app_id", app_id,
        KV_INT, "sw_state", reported
    );
}

void rfswitch_op(uint8_t app_id) {
    for (int i = 0; i < 8; i++) {
        gpio_put(rfswitch.pins[i], (rfswitch.commanded_state >> i) & 0x1);
    }
    if (rfswitch.in_transition && time_reached(rfswitch.transition_end)) {
        rfswitch.reported_state = rfswitch.commanded_state;
        rfswitch.in_transition = false;
    }
}
