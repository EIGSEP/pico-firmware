#ifndef EIGSEP_COMMAND_H
#define EIGSEP_COMMAND_H

#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdint.h>
#include "cJSON.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BUFFER_SIZE 256

typedef enum {
    KV_STR,
    KV_INT,
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
