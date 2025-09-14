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

static inline int distCW(int start, int end, int sensor_max) {
    int d = end - start;
    if (d < 0) d += sensor_max + 1;
    return d;
}

static inline uint16_t mapSensor(uint16_t X, uint16_t min, uint16_t max, Direction dir, uint16_t outMax, int sensor_max) {
    int full = sensor_max + 1;
    int distCWVal = distCW(min, max, sensor_max);
    int distCCWVal = full - distCWVal;

    if (distCWVal == 0 || distCCWVal == 0) {
        return 0; 
    }

    long pos, dist;
    if (dir == CW) {
        pos  = distCW(min, X, sensor_max);
        dist = distCWVal;
    } else { // CCW
        pos  = distCW(X, min, sensor_max);
        dist = distCCWVal;
    }

    long Y = pos * outMax / dist;
    if (Y < 0) Y = 0;
    if (Y > outMax) Y = outMax;
    return (uint16_t)Y;
}


#endif //UTILS_H
