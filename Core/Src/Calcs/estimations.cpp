#include "estimations.h"
#include "../Constants/constants.h"
#include <stdio.h>

#define kRX 0
#define kPX 0
#define kRY 0
#define kPY 0

void Estimator::calculateEstimations(Gyro *g, MTF02P* mtf02p, float dt)
{
    MICOLINK_PAYLOAD_RANGE_SENSOR_t optFlow = mtf02p->payload;
    k2d->filter((g->inertialAccel[2] - 1) * 9.81 * 1000, optFlow.distance, dt);
    positionZ = k2d->s00;
    velZ = k2d->s10;
    filterVelZ->input(velZ);
    velZ = constrain(filterVelZ->output(), -1200, 1200);
    isPositioningAvailable = true;

//    printf("%.3f %d %d\n", velZ/10, 50, -50);

    if (optFlow.flow_quality < 70 || positionZ < 200)
    {
        isPositioningAvailable = false;
        estimatedVelX = 0;
        estimatedVelY = 0;
        estimatedPosX = 0;
        estimatedPosY = 0;
        return;
    }

    float height = optFlow.distance/1000.0;

    // Just fixing axis since mtf02 it's not positioned in the suggested orientation as it wouldn't fit frame
    flowX = -optFlow.flow_vel_x;
    flowY = -optFlow.flow_vel_y;

    float velX = flowX - (kPX * g->gyro[0] + kRX * g->gyro[1]);
    float velY = flowY - (kPY * g->gyro[0] + kRY * g->gyro[1]);

    velX = (velX * height) / 100.0;
    velY = (velY * height) / 100.0;

    filterFLowX->input(flowX);
	velX = filterFLowX->output();
	filterFLowY->input(flowY);
	velY = filterFLowY->output();

//    kalman1d(estimatedVelY, velYUncertainty, g->inertialAccel[1] * 10, 10, velY, 3, &estimatedVelY, &velYUncertainty, dt);
//    kalman1d(estimatedVelX, velXUncertainty, g->inertialAccel[0] * 9.81, 10, velX, 3, &estimatedVelX, &velXUncertainty, dt);

//    printf("%.3f %.3f %d %d\n", estimatedVelX, estimatedVelY, -100, 100);
//
//    estimatedPosX += estimatedVelX * dt;
//    estimatedPosY += estimatedVelY * dt;
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
