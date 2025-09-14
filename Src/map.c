//
// Created by dextercai on 2025/9/14.
//
#include "map.h"
int mask_10bit = (1 << 10) - 1;

uint16_t map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max) {
    if (in_min == in_max) {
        return out_min;
    }
    int32_t numerator   = (int32_t)(x - in_min) * (out_max - out_min);
    int32_t denominator = (int32_t)(in_max - in_min);
    int32_t result      = numerator / denominator + out_min;
    // 如果需要限制输出范围，加上 clamp
    if (result < (out_min < out_max ? out_min : out_max))
        result = (out_min < out_max ? out_min : out_max);
    if (result > (out_min > out_max ? out_min : out_max))
        result = (out_min > out_max ? out_min : out_max);
    return (uint16_t)result;
}