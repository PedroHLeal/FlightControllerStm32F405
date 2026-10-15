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
    this->measurementDelaySteps = measurementDelaySteps < HISTORY ? measurementDelaySteps : HISTORY - 1;
    H = {0, 1};   // the measurement is the velocity state
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
    pos = vel = 0;
    for (int i = 0; i < HISTORY; i++)
        velHist[i] = 0;
    histIdx = 0;
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

    // predicted velocity enters the history; update() overwrites it with the corrected one
    lastHistIdx = histIdx;
    velHist[lastHistIdx] = S(1, 0);
    histIdx = (histIdx + 1) % HISTORY;

    pos = S(0, 0);
    vel = S(1, 0);
}

void Kalman2d::update(float velocityMeasurement)
{
    // the measurement describes how fast we were measurementDelaySteps ago, not now
    float predictedThen = measurementDelaySteps == 0 ? S(1, 0)
            : velHist[(lastHistIdx - measurementDelaySteps + HISTORY) % HISTORY];
    float innovation = velocityMeasurement - predictedThen;
    float innovationVariance = P(1, 1) + measurementAccuracy * measurementAccuracy;
    K = P * ~H * (1.0f / innovationVariance);
    S = S + K * innovation;
    P = (I - K * H) * P;

    // corrected velocity goes into the history, so later corrections are not applied twice
    velHist[lastHistIdx] = S(1, 0);

    pos = S(0, 0);
    vel = S(1, 0);
}

float Kalman2d::filter(float input, float velocityMeasurement, float dt)
{
    predict(input, dt);
    update(velocityMeasurement);
    return vel;
}
