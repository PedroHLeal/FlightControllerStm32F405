#include "estimations.h"
#include "../Constants/constants.h"
#include <stdio.h>

void Estimator::calculateEstimations(Gyro *g, MTF02P* mtf02p, float dt)
{
    MICOLINK_PAYLOAD_RANGE_SENSOR_t optFlow = mtf02p->payload;
    k2d->filter((g->inertialAccel[2] - 1) * 9.81 * 1000, optFlow.distance, dt);
    positionZ = k2d->s00;
    velZ = k2d->s10;
    isPositioningAvailable = true;

//    printf("%.3f %d %d\n", velZ, 50, -50);

    if (optFlow.flow_quality < 70 || positionZ < 200)
    {
        isPositioningAvailable = false;
        estimatedVelX = 0;
        estimatedVelY = 0;
        estimatedPosX = 0;
        estimatedPosY = 0;
        return;
    }

    float flowX = optFlow.flow_vel_x*(optFlow.distance/1000.0);
    float flowY = optFlow.flow_vel_y*(optFlow.distance/1000.0);

    float velY = -(flowX - g->gyro[0] * 0.5);
    float velX = -(flowY + g->gyro[1] * 0.5);

    filterFLowX->input(velX);
	velX = filterFLowX->output();
	filterFLowY->input(velY);
	velY = filterFLowY->output();

    kalman1d(estimatedVelY, velYUncertainty, g->inertialAccel[1] * 10, 10, velY, 3, &estimatedVelY, &velYUncertainty, dt);
    kalman1d(estimatedVelX, velXUncertainty, g->inertialAccel[0] * 9.81, 10, velX, 3, &estimatedVelX, &velXUncertainty, dt);

//	estimatedVelX = 0.98 * (estimatedVelX + g->inertialAccel[0] * 9.81 * dt) + 0.02 * velX;
//	estimatedVelY = 0.98 * (estimatedVelY + g->inertialAccel[1] * 9.81 * dt) + 0.02 * velY;

//    printf("%.3f %.3f %d %d\n", estimatedVelX, estimatedVelY, -100, 100);

    estimatedPosX += estimatedVelX * dt;
    estimatedPosY += estimatedVelY * dt;
}

void Estimator::resetEstimations()
{
    estimatedVelX = 0;
    estimatedVelY = 0;
    estimatedVelZ = 0;
    estimatedPosX = 0;
    estimatedPosY = 0;
    positionZ = 0;
    velZ = 0;
}
