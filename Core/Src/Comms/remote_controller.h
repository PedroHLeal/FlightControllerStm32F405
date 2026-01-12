#ifndef REMOTE_CONTROLLER
#define REMOTE_CONTROLLER

typedef struct
{
    float targetThrottle = 0, setPointPitch = 0, setPointRoll = 0;
    float desiredHeight = 0;
    int armed = 0;
} ControllerReadings;

#endif
