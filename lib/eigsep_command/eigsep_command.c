#include "eigsep_command.h"

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

            case KV_INT: 
                {
                    int int_val = va_arg(ap, int);
                    cJSON_AddNumberToObject(reply, key, int_val);
                    str = NULL; /* Skip the string addition */
                }
                break;

            case KV_FLOAT:
                {
                    double float_val = va_arg(ap, double);
                    cJSON_AddNumberToObject(reply, key, float_val);
                    str = NULL; /* Skip the string addition */
                }
                break;

            case KV_BOOL:
                {
                    int bool_val = va_arg(ap, int);
                    cJSON_AddBoolToObject(reply, key, bool_val);
                    str = NULL; /* Skip the string addition */
                }
                break;

            case KV_BYTES:
                /* For bytes, we expect a pointer to data and length */
                str = va_arg(ap, const char *);
                break;

            case KV_FLOAT:
                /* float is promoted to double in varargs */
                sprintf(buf, "%.6f", va_arg(ap, double));
                str = buf;
                break;
        }

        if (str != NULL) {
            cJSON_AddStringToObject(reply, key, str);
        }
    }
    va_end(ap);

    char *out = cJSON_PrintUnformatted(reply);
    printf("%s\n", out);
    cJSON_free(out);
    cJSON_Delete(reply);
}
