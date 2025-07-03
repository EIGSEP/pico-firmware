#include "eigsep_command.h"

void pack_and_encode_uint8(uint8_t value, char *out, size_t out_size) {
    uint8_t bin[1];
    bin[0] = value & 0xFF;
    base64_encode(bin, 1, out, out_size);
}

void pack_and_encode_uint16(uint16_t value, char *out, size_t out_size) {
    uint8_t bin[2];
    bin[0] = value & 0xFF;
    bin[1] = (value >> 8) & 0xFF;
    base64_encode(bin, 2, out, out_size);
}

void pack_and_encode_uint32(uint32_t value, char *out, size_t out_size) {
    uint8_t bin[4];
    bin[0] = value & 0xFF;
    bin[1] = (value >> 8) & 0xFF;
    bin[2] = (value >> 16) & 0xFF;
    bin[3] = (value >> 24) & 0xFF;
    base64_encode(bin, 4, out, out_size);
}

void pack_and_encode_float(float value, char *out, size_t out_size) {
    uint8_t bin[4];
    memcpy(bin, &value, sizeof(float));
    base64_encode(bin, 4, out, out_size);
}

void pack_and_encode_bytes(const uint8_t *data, size_t data_len, char *out, size_t out_size) {
    base64_encode(data, data_len, out, out_size);
}

void handle_json_command(const char *line, uint32_t *cadence_ms) {
    cJSON *json = cJSON_Parse(line);
    if (json) {
        const cJSON *cmd = cJSON_GetObjectItem(json, "command");
        if (cmd && strcmp(cmd->valuestring, "set_cadence") == 0) {
            const cJSON *value = cJSON_GetObjectItem(json, "ms");
            if (value && cJSON_IsNumber(value)) {
                *cadence_ms = value->valueint;
            }
        }
        cJSON_Delete(json);
    } else {
        printf("{\"error\": \"Invalid JSON\"}\n");
    }
}

void send_json(unsigned count, ...)
{
    va_list ap;
    va_start(ap, count);

    cJSON *reply = cJSON_CreateObject();
    for(unsigned i = 0; i < count; ++i) {
        /* retrieve the tag as an int, then cast */
        int tag_i = va_arg(ap, int);
        kv_type_t tag = (kv_type_t)tag_i;

        const char *key = va_arg(ap, const char *);
        char buf[64];
        size_t len = sizeof(buf);
        const char *str = NULL;

        switch(tag) {
            case KV_STR:
                str = va_arg(ap, const char *);
                break;

            case KV_UINT8: {
                /* uint16_t is promoted to int as well */
                uint8_t v = (uint8_t)va_arg(ap, int);
                pack_and_encode_uint8(v, buf, len);
                str = buf;
                break;
            }

            case KV_UINT16: {
                /* uint16_t is promoted to int as well */
                uint16_t v = (uint16_t)va_arg(ap, int);
                pack_and_encode_uint16(v, buf, len);
                str = buf;
                break;
            }

            case KV_UINT32: {
                /* uint32_t on a 32-bit int/long platform is an unsigned int */
                uint32_t v = va_arg(ap, uint32_t);
                pack_and_encode_uint32(v, buf, len);
                str = buf;
                break;
            }

            case KV_FLOAT: {
                /* float is promoted to double */
                double fv = va_arg(ap, double);
                pack_and_encode_float((float)fv, buf, len);
                str = buf;
                break;
            }

            case KV_BYTES: {
                const uint8_t *data = va_arg(ap, const uint8_t *);
                size_t data_len     = va_arg(ap, size_t);
                pack_and_encode_bytes(data, data_len, buf, len);
                str = buf;
                break;
            }
        }

        cJSON_AddStringToObject(reply, key, str);
    }
    va_end(ap);

    char *out = cJSON_PrintUnformatted(reply);
    printf("%s\n", out);
    cJSON_free(out);
    cJSON_Delete(reply);
}
