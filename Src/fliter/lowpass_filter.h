//
// Created by dextercai on 25-8-4.
//

#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H
#include <stdint.h>

typedef struct {
    float alpha;     // 滤波系数 ∈ (0,1)
    float value;     // 当前估计值
    uint8_t init;    // 是否已初始化（避免起始突变）
} LowPassFilter;

void LowPassFilter_Init(LowPassFilter *lp, float alpha, float initial);
float LowPassFilter_Update(LowPassFilter *lp, float input);

#endif //LOWPASS_FILTER_H
