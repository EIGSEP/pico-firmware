#ifndef EIGSEP_COMMAND_H
#define EIGSEP_COMMAND_H

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "cJSON.h"
#include "base64.h"

#define BUFFER_SIZE 256

typedef enum {
    KV_STR,
    KV_INT,
    KV_FLOAT,
    KV_BYTES
} kv_type_t;

void pack_and_encode_uint8(uint8_t value, char *out, size_t out_size);
void pack_and_encode_uint16(uint16_t value, char *out, size_t out_size);
void pack_and_encode_uint32(uint32_t value, char *out, size_t out_size);
void pack_and_encode_float(float value, char *out, size_t out_size);
void pack_and_encode_bytes(const uint8_t *data, size_t data_len, char *out, size_t out_size);
void handle_json_command(const char *line, uint32_t *cadence_ms);
void send_json(unsigned count, ...);

#endif  // EIGSEP_COMMAND_H
