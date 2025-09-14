//
// Created by dextercai on 25-8-13.
//

#include "utils.h"

// uint16 -> uint8[2]
void Uint16ToUint8Array(uint16_t value, uint8_t arr[2], uint8_t bigEndian) {
    if (bigEndian) {
        arr[0] = (uint8_t)(value >> 8);   // 高字节
        arr[1] = (uint8_t)(value & 0xFF); // 低字节
    } else {
        arr[0] = (uint8_t)(value & 0xFF); // 低字节
        arr[1] = (uint8_t)(value >> 8);   // 高字节
    }
}

// uint8[2] -> uint16
uint16_t Uint8ArrayToUint16(const uint8_t arr[2], uint8_t bigEndian) {
    if (bigEndian) {
        return ((uint16_t)arr[0] << 8) | arr[1];
    } else {
        return ((uint16_t)arr[1] << 8) | arr[0];
    }
}