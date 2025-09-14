//
// Created by dextercai on 25-6-28.
//

#include "averageFilter.h"

#include <stdlib.h>
#include <main.h>
#include <string.h>

SlipAvgFilter_t SlipAvgFilterCreate(uint16_t cap)
{
    if (cap < 1)
    {
        cap = 1;
    }
    const SlipAvgFilter_t ret = {
        .cap = cap,
        .idx = 0,
        .len = 0,
        .sum = 0,
        .avg = 0,
        .data = malloc(sizeof(uint16_t) * cap),
    };
    if (ret.data != NULL) {
        memset(ret.data, 0, sizeof(uint16_t) * cap);
    }
    return ret;
}

uint16_t SlipAvgFilterGetAvgWithInput(SlipAvgFilter_t* filter, uint16_t input)
{
    filter->len += 1;
    filter->len = filter->len > filter->cap ? filter->cap : filter->len;
    filter->sum -= filter->data[filter->idx];
    filter->data[filter->idx] = input;
    filter->idx = (filter->idx + 1) % filter->cap;
    filter->sum += input;
    filter->avg = filter->sum / filter->len;
    return filter->avg;
}


uint16_t SlipAvgFilterGetAvg(SlipAvgFilter_t* filter)
{
    return filter->avg;
}