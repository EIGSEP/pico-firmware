#ifndef RFSWITCH_H
#define RFSWITCH_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/time.h"
#include "hardware/gpio.h"
#include "eigsep_command.h"

// The RF switch PCB holds two AT28BV64B EEPROMs wired as a live lookup
// table driving three ADGM1004 switches plus the noise-diode bias: the
// byte stored at the address presented on A0..A4 drives the switch
// control lines. Selecting a path therefore means driving a 5-bit
// address onto GP8..GP12; this firmware never touches the EEPROM bus
// itself. Table ground truth: eeprom_api/program_paths/program_paths.c
#define RFSWITCH_A0_PIN      8   // A_i = GP(RFSWITCH_A0_PIN + i)
#define RFSWITCH_ADDR_LINES  5

// Addresses 0x00-0x0F are burned with the path table below; 0x10-0x1F
// hold 0xFF, which would close every switch input at once and enable
// the noise diode, so commands at or above this bound are rejected.
#define RFSWITCH_NUM_PATHS  16

// Burned path table, for reference; firmware logic only needs the
// RFSWITCH_NUM_PATHS bound. Names mirror PicoRFSwitch.PATHS in picohost.
typedef enum {
    RF_PATH_LNA_FEED        = 0x00,  // fail-safe default (address lines low)
    RF_PATH_VNA_CAL_LOAD    = 0x01,
    RF_PATH_VNA_CAL_OPEN    = 0x02,
    RF_PATH_VNA_CAL_SHORT   = 0x03,
    RF_PATH_VNA_FEED        = 0x04,
    RF_PATH_VNA_NOISE_ON    = 0x05,
    RF_PATH_VNA_NOISE_OFF   = 0x06,
    RF_PATH_VNA_LNA         = 0x07,
    RF_PATH_VNA_AMBHOT      = 0x08,
    RF_PATH_VNA_SPARE1      = 0x09,
    RF_PATH_VNA_SPARE2      = 0x0A,
    RF_PATH_LNA_NOISE_ON    = 0x0B,
    RF_PATH_LNA_NOISE_OFF   = 0x0C,
    RF_PATH_LNA_AMBHOT      = 0x0D,
    RF_PATH_LNA_SPARE1      = 0x0E,
    RF_PATH_LNA_SPARE2      = 0x0F,
} rf_path_t;

// Sentinel sw_state value reported while the physical switch is still
// settling. Downstream consumers treat this as "state unknown".
#define SW_STATE_UNKNOWN (-1)

// Covers EEPROM read access (~200 ns) plus ADGM1004 MEMS actuation
// (~100 us) with generous margin.
#define SWITCH_SETTLE_MS 20


typedef struct {
    int commanded_state;        // path address driven to A0..A4 right now
    int reported_state;         // last state the firmware trusts as settled
    bool in_transition;         // true while waiting for settle timer
    absolute_time_t transition_end;
} RFSwitch;

// report rfswitch status
void rfswitch_init(uint8_t);
void rfswitch_server(uint8_t, const char *);
void rfswitch_op(uint8_t);
void rfswitch_status(uint8_t);

#endif // RFSWITCH_H
