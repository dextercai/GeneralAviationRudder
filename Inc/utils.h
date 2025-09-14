//
// Created by dextercai on 25-8-13.
//

#ifndef UTILS_H
#define UTILS_H
#include <stdint.h>
#include <types.h>

// uint16 -> uint8[2]
void Uint16ToUint8Array(uint16_t value, uint8_t arr[2], uint8_t bigEndian);

// uint8[2] -> uint16
uint16_t Uint8ArrayToUint16(const uint8_t arr[2], uint8_t bigEndian) ;

#endif //UTILS_H
