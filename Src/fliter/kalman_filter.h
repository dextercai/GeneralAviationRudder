//
// Created by dextercai on 25-8-3.
//

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include "stm32f1xx_hal.h"

typedef struct {
    float q;     // process noise covariance
    float r;     // measurement noise covariance
    float x;     // value (estimated)
    float p;     // estimation error covariance
    float k;     // kalman gain
} KalmanFilter;

void KalmanFilter_Init(KalmanFilter *kf, float q, float r, float init_value);
float KalmanFilter_Update(KalmanFilter *kf, float measurement);

#endif //KALMAN_FILTER_H
