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
// Flow sensor position relative to the centre of rotation, cm, body frame (+X right, +Y forward).
// Yawing makes the sensor translate at w x r, which the flow reports as body velocity.
#define FLOW_OFFSET_X_CM -0.2f
#define FLOW_OFFSET_Y_CM 5.0f
#define FLOW_YAW_SIGN 1       // flip if yawing in place makes the compensated velocity larger, not smaller
// Flow is only trusted above this height (mm). Near the floor the angular flow per cm of motion
// is large, downwash stirs the ground and the lens sees too little texture; takeoff and landing
// stay in angle mode. One hard line, on purpose: easier to debug than a band.
#define FLOW_MIN_HEIGHT_MM 500
// Z is ToF only: velocity is the derivative of the range, no accelerometer. A reading that moves
// more than TOF_STEP_MM is terrain (a table edge) or garbage: ignored for velocity, the previous
// velocity is held and the altitude simply follows the sensor. Real motion moves the ToF a few
// cm per update. A reading that has not changed for TOF_STILL_MS means the drone is not moving
// vertically, so the held velocity decays to zero rather than sticking at the last derivative.
#define TOF_STEP_MM 150
#define TOF_STILL_MS 100
// The hold engages only from a settled state: a climb through the height line puts thrust well
// above 1 g, and any attitude error leaks that into a phantom horizontal velocity the loop
// would then fight. Vertical speed must stay under this for the FLOW_GOOD_MS before engaging.
#define FLOW_ENGAGE_MAX_VZ_MMS 150

void Estimator::calculateEstimations(Gyro *g, MTF02P* mtf02p, float dt, long us)
{
    MICOLINK_PAYLOAD_RANGE_SENSOR_t optFlow = mtf02p->payload;
    // ---- Z: ToF only ----
    float d = (float) optFlow.distance;
    if (tofSinceChangeMs < 100000)
        tofSinceChangeMs++;
    if (d != 0 && d != lastTofMm)
    {
        if (lastTofMm != 0)
        {
            float delta = d - lastTofMm;
            if (delta <= TOF_STEP_MM && delta >= -TOF_STEP_MM)
                rawVelZ = delta / (tofSinceChangeMs * 0.001f);   // mm/s
            // else: a step, keep the previous velocity
        }
        lastTofMm = d;
        tofSinceChangeMs = 0;
        positionZ = d;
    }
    else if (tofSinceChangeMs > TOF_STILL_MS)
        rawVelZ = 0;
    velZ = rawVelZ;
    filterVelZ->input(velZ, us);
    velZ = constrain(filterVelZ->output(), -1200, 1200);
    // computed before the flow gate so the bench diagnostics see it on the floor too
    accX = FUSION_ACC_X_SIGN * g->inertialAccel[FUSION_ACC_X_AXIS] * 981.0f;
    accY = FUSION_ACC_Y_SIGN * g->inertialAccel[FUSION_ACC_Y_AXIS] * 981.0f;

    // on the raw ToF: after a terrain step positionZ is absolute, the flow cares about the floor
    flowHeightOk = optFlow.distance >= FLOW_MIN_HEIGHT_MM;
    bool flowOk = optFlow.flow_quality >= FLOW_MIN_QUALITY && flowHeightOk;
    bool settled = velZ > -FLOW_ENGAGE_MAX_VZ_MMS && velZ < FLOW_ENGAGE_MAX_VZ_MMS;
    if (flowOk)
    {
        flowBadMs = 0;
        if (!settled)
            flowGoodMs = 0;   // only matters before engagement: an engaged hold keeps running
        else if (flowGoodMs < FLOW_GOOD_MS)
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
            estimatedVelX = kX->vel;
            estimatedVelY = kY->vel;
            integratePosition(g, dt);
            return;
        }
        isPositioningAvailable = false;
        coasting = false;
        resetXY();
        return;
    }

    coasting = false;

    // hysteresis: the estimator runs (and converges) on good flow for FLOW_GOOD_MS before the
    // hold may engage, so it starts from a settled estimate and a flicker cannot re-engage it
    if (!isPositioningAvailable && flowGoodMs >= FLOW_GOOD_MS)
        isPositioningAvailable = true;

    float height = optFlow.distance/1000.0;

    // sensor frame: remove the flow the body's own rotation induces. The sensor reports
    // late, so the gyro is delayed to match it.
    float g0, g1, g2;
    g->gyroDelayed(FLOW_GYRO_DELAY_MS, &g0, &g1, &g2);
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

    // remove the sensor's own translation from yawing about the centre of rotation (v = w x r)
    float wz = g2 * DEG_TO_RAD * FLOW_YAW_SIGN;
    flowVelX += wz * FLOW_OFFSET_Y_CM;
    flowVelY -= wz * FLOW_OFFSET_X_CM;
    filterFLowX->input(flowVelX, us);
    velX = filterFLowX->output();
    filterFLowY->input(flowVelY, us);
    velY = filterFLowY->output();

    // the raw flow velocity is the measurement: the filter does the smoothing, and the
    // delayed horizon inside it accounts for the sensor latency
    kX->filter(accX, flowVelX, dt);
    kY->filter(accY, flowVelY, dt);
    if (FUSION_USE_ACCEL)
    {
        estimatedVelX = kX->vel;
        estimatedVelY = kY->vel;
    }
    else
    {
        estimatedVelX = velX;
        estimatedVelY = velY;
    }
    integratePosition(g, dt);
}

// body velocity -> arm-heading frame, then integrate: the hold point must not rotate with the drone
void Estimator::integratePosition(Gyro *g, float dt)
{
    yawDeg = g->rotationAngle[2];
    float psi = FRAME_YAW_SIGN * yawDeg * DEG_TO_RAD;
    float c = cosf(psi), s = sinf(psi);
    estimatedPosX += (c * estimatedVelX - s * estimatedVelY) * dt;
    estimatedPosY += (s * estimatedVelX + c * estimatedVelY) * dt;
}

void Estimator::resetXY()
{
    estimatedVelX = 0;
    estimatedVelY = 0;
    estimatedPosX = 0;
    estimatedPosY = 0;
    kX->reset(100, 100);
    kY->reset(100, 100);
}

void Estimator::resetEstimations()
{
    resetXY();
    estimatedVelZ = 0;
    positionZ = 0;
    velZ = 0;
    tofSinceChangeMs = 0;
    lastTofMm = 0;
    rawVelZ = 0;
}
