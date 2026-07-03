#ifndef EIGSEP_COMMAND_H
#define EIGSEP_COMMAND_H

#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "cJSON.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BUFFER_SIZE 256

typedef enum {
    KV_STR,
    KV_INT,
    /* A NaN (or infinite) KV_FLOAT value is emitted as JSON null —
       cJSON's print_number substitutes "null" for non-finite doubles.
       Apps use this to report "no valid reading" for a numeric field
       (e.g. tempctrl T_now while the sensor data is invalid). */
    KV_FLOAT,
    KV_BYTES,
    KV_BOOL
} kv_type_t;

void handle_json_command(const char *line, uint32_t *cadence_ms);
void send_json(unsigned count, ...);

#ifdef __cplusplus
}
#endif

#endif  // EIGSEP_COMMAND_H
