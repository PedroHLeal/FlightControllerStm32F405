#ifndef FILTERS_H
#define FILTERS_H

#include "BasicLinearAlgebra/BasicLinearAlgebra.h"

void kalman1d(
    float KalmanState,
    float KalmanUncertainty,
    float KalmanInput,
    float KalmanInputAccuracy,
    float KalmanMeasurement,
    float KalmanMeasurementAccuracy,
    float *output,
    float *uncertainty,
    float dt);

// [position, velocity] driven by a measured acceleration and corrected by a position
// measurement that may arrive late: the innovation is formed against the position the
// filter had measurementDelaySteps ago.
class Kalman2d {
    // 2-state (position, velocity) filter: an acceleration drives the prediction, a velocity
    // measurement corrects it. The measurement arrives measurementDelaySteps late, so it is
    // compared against the velocity the filter had back then (a short history), not now.
    // Position is the integral of the corrected velocity; nothing measures it.
private:
    float inputAccuracy, measurementAccuracy;
    static const int HISTORY = 64;
    float velHist[HISTORY] = {};
    int histIdx = 0, lastHistIdx = 0, measurementDelaySteps = 0;
    BLA::Matrix<2, 2, float> F, P, Q, I;
    BLA::Matrix<2, 1, float> G, S, K;
    BLA::Matrix<1, 2, float> H;

public:
    float pos = 0, vel = 0;
    Kalman2d(float inputAccuracy, float measurementAccuracy, int measurementDelaySteps = 0);
    void reset(float posVariance, float velVariance);
    void predict(float input, float dt);
    void update(float velocityMeasurement);
    float filter(float input, float velocityMeasurement, float dt);
};

#endif
