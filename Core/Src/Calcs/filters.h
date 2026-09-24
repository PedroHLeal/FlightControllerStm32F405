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
private:
    float inputAccuracy, measurementAccuracy;
    static const int POS_HISTORY = 64;
    float posHist[POS_HISTORY] = {};
    int posHistIdx = 0, lastHistIdx = 0, measurementDelaySteps = 0;
    BLA::Matrix<2, 2, float> F, P, Q, I;
    BLA::Matrix<2, 1, float> G, S, K;
    BLA::Matrix<1, 2, float> H;

public:
    float pos = 0, vel = 0;
    float s00 = 0, s10 = 0;   // aliases of pos / vel
    Kalman2d(float inputAccuracy, float measurementAccuracy, int measurementDelaySteps = 0);
    void reset(float posVariance, float velVariance);
    void predict(float input, float dt);
    void update(float measurement);
    float filter(float input, float measurement, float dt);
};

#endif
