//
// Created by dextercai on 25-8-4.
//
#include "median_filter.h"
#include <string.h> // for memcpy

// 简单插入排序（只用于小窗口）
static void insertion_sort(uint16_t *arr, uint8_t len) {
    for (uint8_t i = 1; i < len; i++) {
        uint16_t key = arr[i];
        int8_t j = i - 1;
        while (j >= 0 && arr[j] > key) {
            arr[j + 1] = arr[j];
            j--;
        }
        arr[j + 1] = key;
    }
}

void MedianFilter_Init(MedianFilter *mf, uint8_t window_size) {
    if (window_size > MEDIAN_MAX_SIZE) window_size = MEDIAN_MAX_SIZE;
    mf->size = window_size;
    mf->index = 0;
    mf->count = 0;
    memset(mf->buffer, 0, sizeof(mf->buffer));
}

uint16_t MedianFilter_Update(MedianFilter *mf, uint16_t new_value) {
    mf->buffer[mf->index++] = new_value;
    if (mf->index >= mf->size) mf->index = 0;
    if (mf->count < mf->size) mf->count++;

    uint16_t temp[MEDIAN_MAX_SIZE];
    memcpy(temp, mf->buffer, mf->count * sizeof(uint16_t));
    insertion_sort(temp, mf->count);

    return temp[mf->count / 2];  // 返回中值
}


uint16_t MedianFilter_Fetch(const MedianFilter *mf) {
    uint16_t temp[MEDIAN_MAX_SIZE];
    memcpy(temp, mf->buffer, mf->count * sizeof(uint16_t));
    insertion_sort(temp, mf->count);

    return temp[mf->count / 2];  // 返回中值
}
