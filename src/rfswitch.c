#include "rfswitch.h"
#include "pico/stdlib.h"
#include "cJSON.h"
#include <math.h>
#include <stdlib.h>

static RFSwitch rfswitch;

// EEPROM select pins, index = address bit (LSB first). This table IS
// the wiring spec for the harness — see "RF Switch Wiring" in README.md.
static const uint rfswitch_addr_pins[RFSWITCH_ADDR_LINES] = {8, 10, 12, 14, 15};

void rfswitch_init(uint8_t app_id) {
    rfswitch.commanded_state = RF_PATH_LNA_FEED;
    rfswitch.reported_state = RF_PATH_LNA_FEED;
    // Boot starts a transition: the physical switch position is not
    // knowable until the settle timer elapses, even though the address
    // lines drive to 0 (the LNA->Feed fail-safe) immediately.
    rfswitch.in_transition = true;
    rfswitch.transition_end = make_timeout_time_ms(SWITCH_SETTLE_MS);
    for (int i = 0; i < RFSWITCH_ADDR_LINES; i++) {
        gpio_init(rfswitch_addr_pins[i]);
        gpio_set_dir(rfswitch_addr_pins[i], GPIO_OUT);
    }
}


void rfswitch_server(uint8_t app_id, const char *json_str) {
    cJSON *root = cJSON_Parse(json_str);
    if (!root || !cJSON_IsObject(root)) {
        cJSON_Delete(root);
        return;
    }
    cJSON *sw_state_json = cJSON_GetObjectItem(root, "sw_state");
    if (sw_state_json && cJSON_IsNumber(sw_state_json)) {
        double new_state_value = cJSON_GetNumberValue(sw_state_json);
        double integral_part = 0.0;
        bool is_exact_int = (
            isfinite(new_state_value) &&
            modf(new_state_value, &integral_part) == 0.0
        );
        // Addresses >= RFSWITCH_NUM_PATHS hold 0xFF on the EEPROMs
        // (every switch input closed, noise diode on) and must never
        // be presented on the bus.
        bool is_valid_state = (
            is_exact_int &&
            new_state_value >= 0 &&
            new_state_value < RFSWITCH_NUM_PATHS
        );
        // Only re-enter a transition when the commanded state actually
        // changes; repeated commands at the current state are no-ops
        // so we don't smear a settled position into UNKNOWN.
        if (is_valid_state) {
            int new_state = (int)new_state_value;
            if (new_state != rfswitch.commanded_state) {
                rfswitch.commanded_state = new_state;
                rfswitch.transition_end = make_timeout_time_ms(SWITCH_SETTLE_MS);
                rfswitch.in_transition = true;
            }
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
    // Update all five address lines in one register write so the
    // EEPROM never sees a transient intermediate address.
    uint32_t mask = 0, vals = 0;
    for (int i = 0; i < RFSWITCH_ADDR_LINES; i++) {
        mask |= 1u << rfswitch_addr_pins[i];
        if ((rfswitch.commanded_state >> i) & 0x1) {
            vals |= 1u << rfswitch_addr_pins[i];
        }
    }
    gpio_put_masked(mask, vals);
    if (rfswitch.in_transition && time_reached(rfswitch.transition_end)) {
        rfswitch.reported_state = rfswitch.commanded_state;
        rfswitch.in_transition = false;
    }
}
