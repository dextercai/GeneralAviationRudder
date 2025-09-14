//
// Created by dextercai on 25-8-3.
//

#include "kalman_filter.h"

void KalmanFilter_Init(KalmanFilter *kf, float q, float r, float init_value) {
    kf->q = q;
    kf->r = r;
    kf->x = init_value;
    kf->p = 1.0f;
    kf->k = 0.0f;
}

float KalmanFilter_Update(KalmanFilter *kf, float measurement) {
    // Predict
    kf->p += kf->q;

    // Update
    kf->k = kf->p / (kf->p + kf->r);
    kf->x += kf->k * (measurement - kf->x);
    kf->p *= (1.0f - kf->k);

    return kf->x;
}
