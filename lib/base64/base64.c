#include "base64.h"

static const char b64_table[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "0123456789+/";

static const uint8_t b64_decode_table[256] = {
    /* 0–255 initialized to 0xFF (invalid), then valid chars set to 0–63 */
    [0 ... 255] = 0xFF,
    ['A'] =  0, ['B'] =  1, ['C'] =  2, ['D'] =  3, ['E'] =  4,
    ['F'] =  5, ['G'] =  6, ['H'] =  7, ['I'] =  8, ['J'] =  9,
    ['K'] = 10, ['L'] = 11, ['M'] = 12, ['N'] = 13, ['O'] = 14,
    ['P'] = 15, ['Q'] = 16, ['R'] = 17, ['S'] = 18, ['T'] = 19,
    ['U'] = 20, ['V'] = 21, ['W'] = 22, ['X'] = 23, ['Y'] = 24,
    ['Z'] = 25,
    ['a'] = 26, ['b'] = 27, ['c'] = 28, ['d'] = 29, ['e'] = 30,
    ['f'] = 31, ['g'] = 32, ['h'] = 33, ['i'] = 34, ['j'] = 35,
    ['k'] = 36, ['l'] = 37, ['m'] = 38, ['n'] = 39, ['o'] = 40,
    ['p'] = 41, ['q'] = 42, ['r'] = 43, ['s'] = 44, ['t'] = 45,
    ['u'] = 46, ['v'] = 47, ['w'] = 48, ['x'] = 49, ['y'] = 50,
    ['z'] = 51,
    ['0'] = 52, ['1'] = 53, ['2'] = 54, ['3'] = 55, ['4'] = 56,
    ['5'] = 57, ['6'] = 58, ['7'] = 59, ['8'] = 60, ['9'] = 61,
    ['+'] = 62, ['/'] = 63,
};

int base64_encode(const uint8_t *in, size_t in_len, char *out, size_t *out_len) {
    if (!in || !out || !out_len) return -1;
    size_t i = 0, o = 0;
    while (i < in_len) {
        uint32_t buf = in[i++] << 16;
        if (i < in_len) buf |= in[i++] << 8;
        if (i < in_len) buf |= in[i++];
        // produce four 6-bit values
        out[o++] = b64_table[(buf >> 18) & 0x3F];
        out[o++] = b64_table[(buf >> 12) & 0x3F];
        out[o]   = (i - 1 > in_len) ? '=' : b64_table[(buf >> 6) & 0x3F];
        o++;
        out[o]   = (i > in_len) ? '=' : b64_table[buf & 0x3F];
        o++;
    }
    out[o] = '\0';
    *out_len = o;
    return 0;
}

int base64_decode(const char *in, size_t in_len, uint8_t *out, size_t *out_len) {
    if (!in || !out || !out_len || (in_len % 4)) return -1;
    size_t i = 0, o = 0;
    while (i < in_len) {
        uint32_t buf = 0;
        int pads = 0;
        for (int j = 0; j < 4; j++) {
            char c = in[i++];
            if (c == '=') {
                buf <<= 6;
                pads++;
            } else {
                uint8_t v = b64_decode_table[(uint8_t)c];
                if (v == 0xFF) return -2;  // invalid character
                buf = (buf << 6) | v;
            }
        }
        // extract three bytes
        out[o++] = (buf >> 16) & 0xFF;
        if (pads < 2) out[o++] = (buf >> 8) & 0xFF;
        if (pads < 1) out[o++] = buf & 0xFF;
    }
    *out_len = o;
    return 0;
}
