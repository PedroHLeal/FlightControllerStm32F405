#include "pid.h"
#include "Sensors/Gyro.h"
#include "Constants/constants.h"
#include <math.h>
#include <stdio.h>

void calculatePidPosZ(DronePosition *p, ControllerReadings r, Estimator *e)
{
    p->targetVelZ = (r.targetThrottle - e->positionZ / 10) * VEL_Z_P;
    // Serial.println(p->targetVelZ);
}

void calculatePidThrottle(DronePosition *p, ControllerReadings *r, Estimator *e)
{
    float targetVel = p->useZPositioning ? p->targetVelZ : r->targetThrottle * 10;
    float throttleError = targetVel - e->velZ / 10;
    p->throttleI += throttleError * THROTTLE_I_GAIN;

    p->throttle = constrain(throttleError * THROTTLE_P_GAIN + p->throttleI + (throttleError - p->previousThrottleError) * THROTTLE_D_GAIN, 0, 140);
    p->previousThrottleError = throttleError;
    //    p->throttle = r->targetThrottle * 2;
}

static float slewToward(float current, float desired, float maxStep)
{
    float d = desired - current;
    if (d > maxStep)
        d = maxStep;
    else if (d < -maxStep)
        d = -maxStep;
    return current + d;
}

void calculatePidPosX(DronePosition *p, Estimator *e)
{
    p->targetVelX = constrain((p->holdPosX - e->estimatedPosX) * POS_P_GAIN, -POS_HOLD_MAX_VEL, POS_HOLD_MAX_VEL);
}

void calculatePidPosY(DronePosition *p, Estimator *e)
{
    p->targetVelY = constrain((p->holdPosY - e->estimatedPosY) * POS_P_GAIN, -POS_HOLD_MAX_VEL, POS_HOLD_MAX_VEL);
}

void calculatePidVelX(DronePosition *p, Estimator *e)
{
    float velXError = p->targetVelX - e->estimatedVelX;
    p->velXI += velXError * VEL_I_GAIN;
    p->velXI = constrain(p->velXI, -VEL_MAX_I, VEL_MAX_I);
    // stick-right is a negative setPointRoll on this frame, so +X (right) needs a negative angle
    p->setPointRoll = -constrain(velXError * VEL_P_GAIN + p->velXI + (velXError - p->previousVelXError) * VEL_D_GAIN, -POS_HOLD_MAX_ANGLE, POS_HOLD_MAX_ANGLE);
    p->previousVelXError = velXError;
}

void calculatePidVelY(DronePosition *p, Estimator *e)
{
    float velYError = p->targetVelY - e->estimatedVelY;
    p->velYI += velYError * VEL_I_GAIN;
    p->velYI = constrain(p->velYI, -VEL_MAX_I, VEL_MAX_I);
    // stick-forward is a positive setPointPitch on this frame, so +Y (forward) needs a positive angle
    p->setPointPitch = constrain(velYError * VEL_P_GAIN + p->velYI + (velYError - p->previousVelYError) * VEL_D_GAIN, -POS_HOLD_MAX_ANGLE, POS_HOLD_MAX_ANGLE);
    p->previousVelYError = velYError;
}

void calculatePidPitch(DronePosition *p, ControllerReadings *r, Gyro *g)
{
    float spp = p->holdBlend * p->setPointPitch + (1.0f - p->holdBlend) * r->setPointPitch;
    p->pitchRate = (spp - g->rotationAngle[0]) * PITCH_ROLL_P_GAIN;
    p->pitchRate = constrain(p->pitchRate, -MAX_ANGLE, MAX_ANGLE);
}

void calculatePidRoll(DronePosition *p, ControllerReadings *r, Gyro *g)
{
    float spr = p->holdBlend * p->setPointRoll + (1.0f - p->holdBlend) * r->setPointRoll;
    p->rollRate = (spr - g->rotationAngle[1]) * PITCH_ROLL_P_GAIN;
    p->rollRate = constrain(p->rollRate, -MAX_ANGLE, MAX_ANGLE);
}

void calculatePidPitchRate(DronePosition *p, ControllerReadings *r, Gyro *g)
{
    float pitchRateError = p->pitchRate - g->gyro[0];

    p->pitchRateI += pitchRateError * PITCH_ROLL_RATE_I_GAIN;
    p->pitchRateI = constrain(p->pitchRateI, -MAX_I_RATE, MAX_I_RATE);

    p->pitch = pitchRateError * PITCH_ROLL_RATE_P_GAIN + p->pitchRateI + (pitchRateError - p->previousPitchRateError) * PITCH_ROLL_RATE_D_GAIN;
    p->pitch = constrain(p->pitch, -MAX_RATE, MAX_RATE);
    p->previousPitchRateError = pitchRateError;
}

void calculatePidRollRate(DronePosition *p, ControllerReadings *r, Gyro *g)
{
    float rollRateError = p->rollRate - g->gyro[1];

    p->rollRateI += rollRateError * PITCH_ROLL_RATE_I_GAIN;
    p->rollRateI = constrain(p->rollRateI, -MAX_I_RATE, MAX_I_RATE);

    p->roll = rollRateError * PITCH_ROLL_RATE_P_GAIN + p->rollRateI + (rollRateError - p->previousRollRateError) * PITCH_ROLL_RATE_D_GAIN;
    p->roll = constrain(p->roll, -MAX_RATE, MAX_RATE);
    p->previousRollRateError = rollRateError;
}

void calculatePidYaw(DronePosition *p, Gyro *g)
{
    p->yaw = g->rotationAngle[2] * YAW_P_GAIN;
}

void runAngleModePid(DronePosition *p, Gyro *gyro, Estimator *estimator, ControllerReadings *readings)
{
    calculatePidThrottle(p, readings, estimator);
    calculatePidPitch(p, readings, gyro);
    calculatePidRoll(p, readings, gyro);
    calculatePidPitchRate(p, readings, gyro);
    calculatePidRollRate(p, readings, gyro);
    calculatePidYaw(p, gyro);
}

// Sticks centred: hold the position captured at release. Sticks deflected: stick is a
// velocity target (cm/s). Flow unusable: fall back to the stick as an angle setpoint.
void runPositionHold(DronePosition *p, Estimator *e, ControllerReadings *r)
{
    // ramp the handover instead of switching the angle setpoint in one tick; while the hold is
    // off, the last hold command is kept and faded out rather than snapped to zero
    float blendTarget = (VELOCITY_HOLD && e->isPositioningAvailable) ? 1.0f : 0.0f;
    p->holdBlend = slewToward(p->holdBlend, blendTarget, 1.0f / HOLD_BLEND_MS);

    if (!VELOCITY_HOLD)
    {
        p->usePositioning = false;
        p->holding = false;
        return;
    }

    p->usePositioning = e->isPositioningAvailable;
    if (!p->usePositioning)
    {
        p->holding = false;
        return;
    }

    bool sticksCentered = fabsf(r->setPointRoll) < 0.5f && fabsf(r->setPointPitch) < 0.5f;
    if (sticksCentered && !POSITION_HOLD)
    {
        p->holding = false;
        p->targetVelX = 0;
        p->targetVelY = 0;
    }
    else if (sticksCentered)
    {
        if (!p->holding)
        {
            p->holding = true;
            p->holdPosX = e->estimatedPosX;
            p->holdPosY = e->estimatedPosY;
            p->velXI = 0;
            p->velYI = 0;
        }
        calculatePidPosX(p, e);
        calculatePidPosY(p, e);
    }
    else
    {
        p->holding = false;
        p->targetVelX = -r->setPointRoll * STICK_VEL_SCALE;
        p->targetVelY = r->setPointPitch * STICK_VEL_SCALE;
    }

    // Slew-limit the target: a stick step would otherwise slam the angle command into its clamp
    // and the loop overshoots by construction. Loop runs at a fixed 1 kHz.
    const float maxStep = VEL_TARGET_SLEW * 0.001f;
    p->slewedVelX = slewToward(p->slewedVelX, p->targetVelX, maxStep);
    p->slewedVelY = slewToward(p->slewedVelY, p->targetVelY, maxStep);
    p->targetVelX = p->slewedVelX;
    p->targetVelY = p->slewedVelY;

    calculatePidVelX(p, e);
    calculatePidVelY(p, e);
}

void resetPid(DronePosition *p)
{
    p->armed = 0;
    p->throttle = 0;
    p->throttleI = 0;
    p->previousThrottleError = 0;
    p->pitch = 0;
    p->pitchI = 0;
    p->previousPitchError = 0;
    p->roll = 0;
    p->rollI = 0;
    p->previousRollError = 0;
    p->pitchRate = 0;
    p->pitchRateI = 0;
    p->previousPitchRateError = 0;
    p->rollRate = 0;
    p->rollRateI = 0;
    p->previousRollRateError = 0;
    p->yaw = 0;
    p->emergencyQuit = 0;
    p->velXI = 0;
    p->velYI = 0;
    p->previousVelXError = 0;
    p->previousVelYError = 0;
    p->targetVelX = 0;
    p->targetVelY = 0;
    p->slewedVelX = 0;
    p->slewedVelY = 0;
    p->holdBlend = 0;
    p->setPointRoll = 0;
    p->setPointPitch = 0;
    p->holding = false;
}
