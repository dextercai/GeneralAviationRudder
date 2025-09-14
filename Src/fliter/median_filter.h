//
// Created by dextercai on 25-8-4.
//

#ifndef MEDIAN_FILTER_H
#define MEDIAN_FILTER_H

#define MEDIAN_MAX_SIZE 24  // 支持最大窗口大小（可根据内存需求调整）
#include <stdint.h>

typedef struct {
    uint16_t buffer[MEDIAN_MAX_SIZE];  // 滑动窗口
    uint8_t size;        // 实际窗口大小
    uint8_t index;       // 当前写入位置
    uint8_t count;       // 已填入数据数量

} MedianFilter;

void MedianFilter_Init(MedianFilter *mf, uint8_t window_size);
uint16_t MedianFilter_Update(MedianFilter *mf, uint16_t new_value);
uint16_t MedianFilter_Fetch(const MedianFilter *mf);

#endif //MEDIAN_FILTER_H
