//
// Created by dextercai on 25-8-4.
//
#include "lowpass_filter.h"

void LowPassFilter_Init(LowPassFilter *lp, float alpha, float initial) {
    lp->alpha = alpha;
    lp->value = initial;
    lp->init = 1;
}

float LowPassFilter_Update(LowPassFilter *lp, float input) {
    if (!lp->init) {
        lp->value = input;
        lp->init = 1;
    } else {
        lp->value = lp->alpha * input + (1.0f - lp->alpha) * lp->value;
    }
    return lp->value;
}