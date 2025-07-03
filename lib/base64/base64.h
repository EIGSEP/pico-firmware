#ifndef BASE64_H
#define BASE64_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/// Calculate the required output buffer size (in bytes) for encoding.
/// For input length `in_len`, the encoded output will be 4 * ceil(in_len/3).
#define BASE64_ENCODE_OUT_SIZE(in_len)  ((((in_len) + 2) / 3) * 4)

/// Calculate the maximum output buffer size for decoding.
/// The decoded data will be at most 3 * (in_len/4).
#define BASE64_DECODE_OUT_SIZE(in_len)  ((((in_len) + 3) / 4) * 3)

/**
 * Encode `in_len` bytes from `in` into Base64, writing into `out`.
 * - `out` must be at least BASE64_ENCODE_OUT_SIZE(in_len)+1 bytes long.
 * - On success, `*out_len` is set to the number of bytes written (excluding NUL),
 *   `out` is NUL-terminated, and the function returns 0.
 * - Returns non-zero on failure (e.g. if invalid arguments).
 */
int base64_encode(const uint8_t *in, size_t in_len, char *out, size_t *out_len);

/**
 * Decode a Base64 string `in` of length `in_len` into `out`.
 * - `out` must be at least BASE64_DECODE_OUT_SIZE(in_len) bytes long.
 * - On success, `*out_len` is set to the number of bytes written and the
 *   function returns 0.
 * - Returns non-zero if `in` contains invalid Base64 characters.
 */
int base64_decode(const char *in, size_t in_len, uint8_t *out, size_t *out_len);

#ifdef __cplusplus
}
#endif

#endif  // BASE64_H
