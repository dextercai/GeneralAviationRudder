//
// Created by dextercai on 25-6-28.
//

#ifndef AVERAGEFILTER_H
#define AVERAGEFILTER_H

#include "stm32f1xx_hal.h"


typedef struct
{
    uint16_t cap;
    uint16_t idx;
    uint16_t len;
    uint64_t sum;
    uint16_t avg;
    uint16_t* data;

} SlipAvgFilter_t;

SlipAvgFilter_t SlipAvgFilterCreate(uint16_t cap);

uint16_t SlipAvgFilterGetAvgWithInput(SlipAvgFilter_t* filter, uint16_t input);
uint16_t SlipAvgFilterGetAvg(SlipAvgFilter_t* filter);

#endif //AVERAGEFILTER_H
