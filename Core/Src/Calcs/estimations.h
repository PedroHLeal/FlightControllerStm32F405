#ifndef ESTIMATIONS_H
#define ESTIMATIONS_H

#include "../Sensors/gyro.h"
#include "../Sensors/mtp02p.h"
#include "filters.h"
#include "DSPFilters/src/Filters.h"

class Estimator {
public:
    float estimatedVelX = 0, estimatedVelY = 0, estimatedVelZ;
    float estimatedPosX = 0, estimatedPosY = 0, estimatedPosZ;
    float velXUncertainty = 0, velYUncertainty = 0, velZUncertainty = 0;
    float positionZ = 0, velZ = 0;
    float flowX = 0, flowY = 0, velX = 0, velY = 0;
    bool isPositioningAvailable = false;
    void calculateEstimations(Gyro* g, MTF02P* mtf02p, float dt);
    void resetEstimations();
	int calibrationCount = 0;
private:
    FilterOnePole *lowPass = new FilterOnePole(LOWPASS, 5);
    FilterOnePole *filterFLowX = new FilterOnePole(LOWPASS, 10);
    FilterOnePole *filterFLowY = new FilterOnePole(LOWPASS, 10);
    FilterOnePole *filterVelZ = new FilterOnePole(LOWPASS, 20);
    Kalman2d *k2d = new Kalman2d(50, 0.05);
};

#endif
