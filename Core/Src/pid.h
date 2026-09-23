#ifndef PID_H
#define PID_H

#include "Comms/hc-06.h"
#include "Sensors/gyro.h"
#include "Calcs/estimations.h"
#include "Drone/drone_position.h"

#define VEL_Z_P 2
#define THROTTLE_P_GAIN 0.2
#define THROTTLE_I_GAIN 0.001
#define THROTTLE_D_GAIN 1.5

#define POS_P_GAIN 0.04

#define VEL_P_GAIN 0.1
#define VEL_I_GAIN 0.0001
#define VEL_D_GAIN 2

// ----------- ANGLE PID -------------
#define PITCH_ROLL_P_GAIN 5
#define MAX_ANGLE 200

// ----------- RATE PID --------------

#define PITCH_ROLL_RATE_P_GAIN 0.05

#define PITCH_ROLL_RATE_I_GAIN 0.00007
#define MAX_I_RATE 5

#define PITCH_ROLL_RATE_D_GAIN 0.08

#define MAX_RATE 25

//------------ YAW PID ---------------

#define YAW_P_GAIN 0.5

void calculatePidPosZ(DronePosition *p, ControllerReadings r, Estimator *e);
void calculatePidThrottle(DronePosition *p, ControllerReadings *r, Estimator *e);
void calculatePidPosX(DronePosition *p, ControllerReadings r, Estimator *e);
void calculatePidPosY(DronePosition *p, ControllerReadings r, Estimator *e);
void calculatePidVelX(DronePosition *p, ControllerReadings r, Estimator *e);
void calculatePidVelY(DronePosition *p, ControllerReadings r, Estimator *e);
void calculatePidPitch(DronePosition *p, ControllerReadings *r, Gyro *g);
void calculatePidRoll(DronePosition *p, ControllerReadings *r, Gyro *g);
void calculatePidPitchRate(DronePosition *p, ControllerReadings *r, Gyro *g);
void calculatePidRollRate(DronePosition *p, ControllerReadings *r, Gyro *g);
void calculatePidYaw(DronePosition *p, Gyro *g);
void runAngleModePid(DronePosition *p, Gyro *g, Estimator *estimator, ControllerReadings *readings);
void runPositionModePid();
void resetPid(DronePosition *p);

#endif
