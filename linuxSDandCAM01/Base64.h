#ifndef BASE64_H
#define BASE64_H

#include <Arduino.h>

// Function to encode data to base64
const char base64_table[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

// Function to encode data in Base64
String base64Encode(const uint8_t* data, size_t length) {
    String encoded;
    encoded.reserve((length * 4 / 3) + 4); // Optimize memory allocation

    uint8_t a, b, c;
    size_t i = 0;

    while (i < length) {
        a = data[i++];
        b = (i < length) ? data[i++] : 0;
        c = (i < length) ? data[i++] : 0;

        encoded += base64_table[a >> 2];
        encoded += base64_table[((a & 0x03) << 4) | (b >> 4)];
        encoded += (i > length + 1) ? '=' : base64_table[((b & 0x0F) << 2) | (c >> 6)];
        encoded += (i > length) ? '=' : base64_table[c & 0x3F];
    }

    return encoded;
}

#endif // BASE64_H