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
                /* uint16_t is promoted to int as well */
                sprintf(buf, "%d", va_arg(ap, int));
                str = buf;
                break;
        }

        cJSON_AddStringToObject(reply, key, str);
    }
    va_end(ap);

    char *out = cJSON_PrintUnformatted(reply);
    printf("%s\n", out);
    cJSON_free(out);
    cJSON_Delete(reply);
}
