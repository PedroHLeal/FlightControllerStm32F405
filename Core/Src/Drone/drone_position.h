/*
 * drone_position.h
 *
 *  Created on: Aug 20, 2025
 *      Author: pedroleal
 */

#ifndef SRC_DRONE_DRONE_POSITION_H_
#define SRC_DRONE_DRONE_POSITION_H_

typedef struct
{
    float armed = 0;

    float targetVelX = 0;
    float velX = 0;
    float velXI = 0;
    float previousVelXError = 0;
    float usePositioning = false;
    float useZPositioning = false;

    float targetVelY = 0;
    float velY = 0;
    float velYI = 0;
    float previousVelYError = 0;

    float setPointPitch = 0;
    float setPointRoll = 0;

    float throttle = 0;
    float throttleI = 0;
    float previousThrottleError = 0;
    float targetVelZ = 0;

    float pitch = 0;
    float pitchI = 0;
    float previousPitchError = 0;

    float roll = 0;
    float rollI = 0;
    float previousRollError = 0;

    float pitchRate = 0;
    float pitchRateI = 0;
    float previousPitchRateError = 0;

    float rollRate = 0;
    float rollRateI = 0;
    float previousRollRateError = 0;

    float yaw = 0;
    bool emergencyQuit = false;
    float debugData = 0;
} DronePosition;

#endif /* SRC_DRONE_DRONE_POSITION_H_ */
