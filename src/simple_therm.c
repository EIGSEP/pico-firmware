#include "simple_therm.h"
#include "temp_simple.h"
#include "hardware/adc.h"
#include "pico/stdlib.h"
#include <math.h>

// On-die temperature sensor is the last ADC input on both RP2040 and
// RP2350 (same definition as the SDK's ADC_TEMPERATURE_CHANNEL_NUM).
#define SIMPLE_THERM_DIE_ADC_INPUT  (NUM_ADC_CHANNELS - 1)

static SimpleThermChannel channels[SIMPLE_THERM_NUM_CHANNELS];
static float die_temp_c = NAN;
static uint32_t last_sample_ms = 0;
static bool sampled_once = false;

// Divider voltage -> (resistance, temperature). Returns false (outputs
// untouched) for an open/shorted divider or an implausible temperature.
static bool voltage_to_temperature(float voltage, float *resistance,
                                   float *temperature) {
    if (!isfinite(voltage) ||
        voltage <= 0.0f ||
        voltage >= THERMISTOR_SUPPLY_VOLTS) {
        return false;
    }

    float r = SIMPLE_THERM_PULLUP_OHMS
        * voltage / (THERMISTOR_SUPPLY_VOLTS - voltage);
    if (!isfinite(r) || r <= 0.0f) {
        return false;
    }

    float log_r = logf(r / SIMPLE_THERM_REF_OHMS);
    float log_r_sq = log_r * log_r;
    float inverse_kelvin = SIMPLE_THERM_SH_A1
        + SIMPLE_THERM_SH_B1 * log_r
        + SIMPLE_THERM_SH_C1 * log_r_sq
        + SIMPLE_THERM_SH_D1 * log_r_sq * log_r;
    if (!isfinite(inverse_kelvin) || inverse_kelvin <= 0.0f) {
        return false;
    }

    float temp_c = 1.0f / inverse_kelvin - 273.15f;
    // NTCLE100E3 operating range (and the fit's validity): -40..+125 C.
    if (!isfinite(temp_c) || temp_c < -40.0f || temp_c > 125.0f) {
        return false;
    }

    *resistance = r;
    *temperature = temp_c;
    return true;
}

static void channel_update(SimpleThermChannel *ch) {
    if (!ch->adc_configured) {
        return;
    }
    ch->voltage = adc_read_avg_voltage(ch->adc_input);
    float r = 0.0f;
    float t = 0.0f;
    if (voltage_to_temperature(ch->voltage, &r, &t)) {
        ch->resistance = r;
        ch->T_now = t;
    } else {
        ch->resistance = NAN;
        ch->T_now = NAN;
    }
}

// RP2040/RP2350 datasheet: T = 27 - (V - 0.706) / 0.001721.
static float read_die_temp_c(void) {
    float v = adc_read_avg_voltage(SIMPLE_THERM_DIE_ADC_INPUT);
    return 27.0f - (v - 0.706f) / 0.001721f;
}

static void sample_all(void) {
    for (int i = 0; i < SIMPLE_THERM_NUM_CHANNELS; i++) {
        channel_update(&channels[i]);
    }
    die_temp_c = read_die_temp_c();
}

void simple_therm_init(uint8_t app_id) {
    for (int i = 0; i < SIMPLE_THERM_NUM_CHANNELS; i++) {
        SimpleThermChannel *ch = &channels[i];
        ch->adc_configured = adc_channel_init(
            SIMPLE_THERM_FIRST_GPIO + i, &ch->adc_input);
        ch->voltage = 0.0f;
        ch->resistance = NAN;
        ch->T_now = NAN;
    }
    // adc_channel_init has already called adc_init().
    adc_set_temp_sensor_enabled(true);
    sample_all();
    last_sample_ms = to_ms_since_boot(get_absolute_time());
    sampled_once = true;
}

// No commands: this app is status-only.
void simple_therm_server(uint8_t app_id, const char *line) {
    (void)app_id;
    (void)line;
}

void simple_therm_op(uint8_t app_id) {
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (sampled_once && (now - last_sample_ms) < SIMPLE_THERM_SAMPLE_MS) {
        return;
    }
    sample_all();
    last_sample_ms = now;
    sampled_once = true;
}

void simple_therm_status(uint8_t app_id) {
    /* 13 KV pairs: 3 device-wide + 3 per channel x 3 channels + die. */
    send_json(13,
        KV_STR,   "sensor_name",  "simple_therm",
        KV_INT,   "app_id",       (int)app_id,
        KV_STR,   "status",       "update",
        KV_FLOAT, "therm0_T",          channels[0].T_now,
        KV_FLOAT, "therm0_voltage",    channels[0].voltage,
        KV_FLOAT, "therm0_resistance", channels[0].resistance,
        KV_FLOAT, "therm1_T",          channels[1].T_now,
        KV_FLOAT, "therm1_voltage",    channels[1].voltage,
        KV_FLOAT, "therm1_resistance", channels[1].resistance,
        KV_FLOAT, "therm2_T",          channels[2].T_now,
        KV_FLOAT, "therm2_voltage",    channels[2].voltage,
        KV_FLOAT, "therm2_resistance", channels[2].resistance,
        KV_FLOAT, "die_T",             die_temp_c
    );
}
