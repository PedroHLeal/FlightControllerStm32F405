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

// ----------- POSITION HOLD (cm, cm/s, deg) -------------
#define VELOCITY_HOLD 1     // 0: never use the flow loops, sticks are angle setpoints (plain angle mode)
#define POSITION_HOLD 1     // 0: sticks centred just nulls velocity (no hold point)
#define POS_P_GAIN 0.6
#define POS_HOLD_MAX_VEL 30
#define POS_HOLD_MAX_ANGLE 15
#define STICK_VEL_SCALE 4   // stick is +-25, so full deflection = 100 cm/s
#define VEL_TARGET_SLEW 200 // cm/s per second the velocity target may change: shapes stick steps and hold engagement
#define HOLD_BLEND_MS 300   // handover ramp between stick angle and velocity-loop angle setpoints

#define VEL_P_GAIN 0.3
#define VEL_I_GAIN 0.0001
#define VEL_D_GAIN 0
#define VEL_MAX_I 2

// ----------- ANGLE PID -------------
#define PITCH_ROLL_P_GAIN 5
#define MAX_ANGLE 200

// ----------- RATE PID --------------

#define PITCH_ROLL_RATE_P_GAIN 0.07

#define PITCH_ROLL_RATE_I_GAIN 0.00007
#define MAX_I_RATE 5

#define PITCH_ROLL_RATE_D_GAIN 0.1

#define MAX_RATE 25

//------------ YAW PID ---------------

#define YAW_P_GAIN 0.5

void calculatePidPosZ(DronePosition *p, ControllerReadings r, Estimator *e);
void calculatePidThrottle(DronePosition *p, ControllerReadings *r, Estimator *e);
void calculatePidPosX(DronePosition *p, Estimator *e);
void calculatePidPosY(DronePosition *p, Estimator *e);
void calculatePidVelX(DronePosition *p, Estimator *e);
void calculatePidVelY(DronePosition *p, Estimator *e);
void calculatePidPitch(DronePosition *p, ControllerReadings *r, Gyro *g);
void calculatePidRoll(DronePosition *p, ControllerReadings *r, Gyro *g);
void calculatePidPitchRate(DronePosition *p, ControllerReadings *r, Gyro *g);
void calculatePidRollRate(DronePosition *p, ControllerReadings *r, Gyro *g);
void calculatePidYaw(DronePosition *p, Gyro *g);
void runAngleModePid(DronePosition *p, Gyro *g, Estimator *estimator, ControllerReadings *readings);
void runPositionHold(DronePosition *p, Estimator *e, ControllerReadings *r);
void resetPid(DronePosition *p);

#endif
