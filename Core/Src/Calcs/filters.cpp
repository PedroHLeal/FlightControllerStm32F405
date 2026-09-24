#include "filters.h"

void kalman1d(
    float KalmanState,
    float KalmanUncertainty,
    float KalmanInput,
    float KalmanInputAccuracy,
    float KalmanMeasurement,
    float KalmanMeasurementAccuracy,
    float *output,
    float *uncertainty,
    float dt)
{
    KalmanState = KalmanState + dt * KalmanInput;
    KalmanUncertainty = KalmanUncertainty + dt * KalmanInputAccuracy * KalmanInputAccuracy;
    float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + KalmanMeasurementAccuracy * KalmanMeasurementAccuracy);
    KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
    KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
    *output = KalmanState;
    *uncertainty = KalmanUncertainty;
}

Kalman2d::Kalman2d(float inputAccuracy, float measurementAccuracy, int measurementDelaySteps)
{
    this->inputAccuracy = inputAccuracy;
    this->measurementAccuracy = measurementAccuracy;
    this->measurementDelaySteps = measurementDelaySteps < POS_HISTORY ? measurementDelaySteps : POS_HISTORY - 1;
    H = {1, 0};
    I = {1, 0,
         0, 1};
    reset(0, 0);
}

void Kalman2d::reset(float posVariance, float velVariance)
{
    S = {0,
         0};
    P = {posVariance, 0,
         0, velVariance};
    pos = vel = s00 = s10 = 0;
    for (int i = 0; i < POS_HISTORY; i++)
        posHist[i] = 0;
    posHistIdx = 0;
    lastHistIdx = 0;
}

void Kalman2d::predict(float input, float dt)
{
    // pos' = vel, vel' = input
    F = {1, dt,
         0, 1};
    G = {0.5f * dt * dt,
         dt};
    Q = G * ~G * inputAccuracy * inputAccuracy;

    S = F * S + G * input;
    P = F * P * ~F + Q;

    // predicted position enters the history; update() overwrites it with the corrected one
    lastHistIdx = posHistIdx;
    posHist[lastHistIdx] = S(0, 0);
    posHistIdx = (posHistIdx + 1) % POS_HISTORY;

    pos = s00 = S(0, 0);
    vel = s10 = S(1, 0);
}

void Kalman2d::update(float measurement)
{
    // the measurement describes where we were measurementDelaySteps ago, not now
    float predictedThen = measurementDelaySteps == 0 ? S(0, 0)
            : posHist[(lastHistIdx - measurementDelaySteps + POS_HISTORY) % POS_HISTORY];
    float innovation = measurement - predictedThen;
    float innovationVariance = P(0, 0) + measurementAccuracy * measurementAccuracy;
    K = P * ~H * (1.0f / innovationVariance);
    S = S + K * innovation;
    P = (I - K * H) * P;

    // corrected position goes into the history, so later corrections are not applied twice
    posHist[lastHistIdx] = S(0, 0);

    pos = s00 = S(0, 0);
    vel = s10 = S(1, 0);
}

float Kalman2d::filter(float input, float measurement, float dt)
{
    predict(input, dt);
    update(measurement);
    return pos;
}
