#include "estimations.h"
#include "../Constants/constants.h"
#include <math.h>
#include <stdio.h>

// Optical flow calibration: paste the block printed by MODE == CALIBRATE_OPTFLOW
#define FLOW_K_X_G0 1.8435f
#define FLOW_K_X_G1 -0.0533f
#define FLOW_K_Y_G0 -0.1250f
#define FLOW_K_Y_G1 -1.8740f
#define FLOW_GYRO_DELAY_MS 50
#define FLOW_MIRROR_X 1
#define FLOW_YAW_OFFSET_DEG 89.1f

void Estimator::calculateEstimations(Gyro *g, MTF02P* mtf02p, float dt, long us)
{
    MICOLINK_PAYLOAD_RANGE_SENSOR_t optFlow = mtf02p->payload;
    k2d->filter((g->inertialAccel[2] - 1) * 9.81 * 1000, optFlow.distance, dt);
    positionZ = k2d->s00;
    velZ = k2d->s10;
    filterVelZ->input(velZ, us);
    velZ = constrain(filterVelZ->output(), -1200, 1200);
    // computed before the flow gate so the bench diagnostics see it on the floor too
    accX = FUSION_ACC_X_SIGN * g->inertialAccel[FUSION_ACC_X_AXIS] * 981.0f;
    accY = FUSION_ACC_Y_SIGN * g->inertialAccel[FUSION_ACC_Y_AXIS] * 981.0f;

    bool flowOk = optFlow.flow_quality >= FLOW_MIN_QUALITY && positionZ >= 200;
    if (flowOk)
    {
        flowBadMs = 0;
        if (flowGoodMs < FLOW_GOOD_MS)
            flowGoodMs++;
    }
    else
    {
        flowGoodMs = 0;
        flowBadMs++;
    }

    if (!flowOk)
    {
        // coast: the accel keeps the estimate honest for a while, so a brief dropout does not
        // drop the controller into angle mode; give up only on a real loss
        if (isPositioningAvailable && FUSION_USE_ACCEL && flowBadMs <= FUSION_COAST_MS)
        {
            coasting = true;
            kX->predict(accX, dt);
            kY->predict(accY, dt);
            estimatedPosX = kX->pos;
            estimatedVelX = kX->vel;
            estimatedPosY = kY->pos;
            estimatedVelY = kY->vel;
            return;
        }
        isPositioningAvailable = false;
        coasting = false;
        resetXY();
        return;
    }

    if (coasting)
    {
        // the flow position resumes from where the estimate is, not from where the flow last was
        coasting = false;
        flowPosX = kX->pos;
        flowPosY = kY->pos;
    }

    // hysteresis: the estimator runs (and converges) on good flow for FLOW_GOOD_MS before the
    // hold may engage, so it starts from a settled estimate and a flicker cannot re-engage it
    if (!isPositioningAvailable && flowGoodMs >= FLOW_GOOD_MS)
        isPositioningAvailable = true;

    float height = optFlow.distance/1000.0;

    // sensor frame: remove the flow the body's own rotation induces. The sensor reports
    // late, so the gyro is delayed to match it.
    float g0, g1;
    g->gyroDelayed(FLOW_GYRO_DELAY_MS, &g0, &g1);
    float sx = optFlow.flow_vel_x - (FLOW_K_X_G0 * g0 + FLOW_K_X_G1 * g1);
    float sy = optFlow.flow_vel_y - (FLOW_K_Y_G0 * g0 + FLOW_K_Y_G1 * g1);
    if (FLOW_MIRROR_X)
        sx = -sx;

    // sensor frame -> body frame (+X right, +Y forward)
    const float c = cosf(FLOW_YAW_OFFSET_DEG * DEG_TO_RAD);
    const float s = sinf(FLOW_YAW_OFFSET_DEG * DEG_TO_RAD);
    flowX = c * sx - s * sy;
    flowY = s * sx + c * sy;

    // flow is 0.01 rad/s, so flow * height[m] is cm/s
    float flowVelX = flowX * height;
    float flowVelY = flowY * height;
    filterFLowX->input(flowVelX, us);
    velX = filterFLowX->output();
    filterFLowY->input(flowVelY, us);
    velY = filterFLowY->output();

    // the unfiltered integral: integration already smooths, and a filter here would only
    // delay the position measurement
    flowPosX += flowVelX * dt;
    flowPosY += flowVelY * dt;

    kX->filter(accX, flowPosX, dt);
    kY->filter(accY, flowPosY, dt);
    if (FUSION_USE_ACCEL)
    {
        estimatedPosX = kX->pos;
        estimatedVelX = kX->vel;
        estimatedPosY = kY->pos;
        estimatedVelY = kY->vel;
    }
    else
    {
        estimatedPosX = flowPosX;
        estimatedVelX = velX;
        estimatedPosY = flowPosY;
        estimatedVelY = velY;
    }

//    kalman1d(estimatedVelY, velYUncertainty, g->inertialAccel[1] * 10, 10, velY, 3, &estimatedVelY, &velYUncertainty, dt);
//    kalman1d(estimatedVelX, velXUncertainty, g->inertialAccel[0] * 9.81, 10, velX, 3, &estimatedVelX, &velXUncertainty, dt);

//    printf("%.3f %.3f %d %d\n", estimatedVelX, estimatedVelY, -100, 100);
//
//    estimatedPosX += estimatedVelX * dt;
//    estimatedPosY += estimatedVelY * dt;
}

void Estimator::resetXY()
{
    estimatedVelX = 0;
    estimatedVelY = 0;
    estimatedPosX = 0;
    estimatedPosY = 0;
    flowPosX = 0;
    flowPosY = 0;
    kX->reset(100, 100);
    kY->reset(100, 100);
}

void Estimator::resetEstimations()
{
    resetXY();
    estimatedVelZ = 0;
    positionZ = 0;
    velZ = 0;
}
