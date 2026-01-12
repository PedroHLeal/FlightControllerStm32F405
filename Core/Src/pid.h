#ifndef PID_H
#define PID_H

#include "Comms/hc-06.h"
#include "Sensors/gyro.h"
#include "Calcs/estimations.h"
#include "Drone/drone_position.h"

#define VEL_Z_P 2
#define THROTTLE_P_GAIN 0.1
#define THROTTLE_I_GAIN 0.001
#define THROTTLE_D_GAIN 1

#define POS_P_GAIN 0.04

#define VEL_P_GAIN 0.1
#define VEL_I_GAIN 0.0001
#define VEL_D_GAIN 2

#define PITCH_ROLL_P_GAIN 10

#define PITCH_ROLL_RATE_P_GAIN 0.03
#define PITCH_ROLL_RATE_I_GAIN 0.0000
#define PITCH_ROLL_RATE_D_GAIN 0.5

#define YAW_P_GAIN 0.5
#define MAX_RATE 20

void calculatePidPosZ(DronePosition *p, ControllerReadings r, Estimator* e);
void calculatePidThrottle(DronePosition *p, ControllerReadings *r, Estimator* e);
void calculatePidPosX(DronePosition *p, ControllerReadings r, Estimator* e);
void calculatePidPosY(DronePosition *p, ControllerReadings r, Estimator* e);
void calculatePidVelX(DronePosition *p, ControllerReadings r, Estimator* e);
void calculatePidVelY(DronePosition *p, ControllerReadings r, Estimator* e);
void calculatePidPitch(DronePosition *p, ControllerReadings *r, Gyro* g);
void calculatePidRoll(DronePosition *p, ControllerReadings *r, Gyro* g);
void calculatePidPitchRate(DronePosition *p, ControllerReadings *r, Gyro* g);
void calculatePidRollRate(DronePosition *p, ControllerReadings *r, Gyro* g);
void calculatePidYaw(DronePosition *p, Gyro* g);
void resetPid(DronePosition* p);

#endif
