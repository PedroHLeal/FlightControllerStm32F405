#ifndef ESTIMATIONS_H
#define ESTIMATIONS_H

#include "../Sensors/gyro.h"
#include "../Sensors/mtp02p.h"
#include "filters.h"
#include "DSPFilters/src/Filters.h"

// X/Y fusion: accel drives short-term velocity, the flow velocity (what the sensor measures)
// corrects it at the delayed horizon. Position is the integral of the fused velocity.
// Which inertialAccel axis/sign is +right (X) and +forward (Y) is verified by hand in
// DEBUG_MODE: slide the drone and the acc column must go positive first, then negative.
#define FUSION_ACC_X_AXIS 0
#define FUSION_ACC_X_SIGN -1
#define FUSION_ACC_Y_AXIS 1
#define FUSION_ACC_Y_SIGN -1
#define FUSION_ACC_NOISE 20.0f   // cm/s^2, about what 1 deg of attitude error leaks
#define FUSION_VEL_NOISE 10.0f   // cm/s, flow velocity; lower = flow corrects faster
#define FUSION_FLOW_DELAY_MS 50  // flow arrives this late (same latency as FLOW_GYRO_DELAY_MS)
#define FUSION_USE_ACCEL 1       // 0: controller uses filtered flow velocity/position (A/B test)
#define FUSION_COAST_MS 500      // after the flow drops, keep predicting on accel alone this long
#define FLOW_GOOD_MS 200         // flow must be usable this long before positioning (re)engages
// Position is integrated in the heading the drone was armed with (gyro yaw, zeroed at arm), so
// yawing does not rotate the hold point. Flip the sign if the bench test below fails:
// slide forward 50 cm, yaw 90 deg in place, slide back along the same floor line -> posE returns to 0.
#define FRAME_YAW_SIGN 1

class Estimator {
public:
    float estimatedVelX = 0, estimatedVelY = 0, estimatedVelZ;
    float estimatedPosX = 0, estimatedPosY = 0, estimatedPosZ;   // X/Y in the arm-heading frame, cm
    float yawDeg = 0;                                             // gyro heading since arm, deg
    float velXUncertainty = 0, velYUncertainty = 0, velZUncertainty = 0;
    float positionZ = 0, velZ = 0;
    float flowX = 0, flowY = 0, velX = 0, velY = 0;   // velX/velY: filtered flow velocity, cm/s
    float accX = 0, accY = 0;                         // horizontal accel fed to the fusion, cm/s^2
    bool isPositioningAvailable = false;
    bool coasting = false;
    int flowBadMs = 0, flowGoodMs = 0;
    bool flowHeightOk = false;          // raw ToF above FLOW_MIN_HEIGHT_MM
    int tofSinceChangeMs = 0;
    float lastTofMm = 0, rawVelZ = 0;
    void calculateEstimations(Gyro* g, MTF02P* mtf02p, float dt, long us);
    void resetEstimations();
	int calibrationCount = 0;
private:
    FilterOnePole *lowPass = new FilterOnePole(LOWPASS, 5);
    FilterOnePole *filterFLowX = new FilterOnePole(LOWPASS, 4);
    FilterOnePole *filterFLowY = new FilterOnePole(LOWPASS, 4);
    FilterOnePole *filterVelZ = new FilterOnePole(LOWPASS, 20);
    Kalman2d *kX = new Kalman2d(FUSION_ACC_NOISE, FUSION_VEL_NOISE, FUSION_FLOW_DELAY_MS);
    Kalman2d *kY = new Kalman2d(FUSION_ACC_NOISE, FUSION_VEL_NOISE, FUSION_FLOW_DELAY_MS);
    void resetXY();
    void integratePosition(Gyro *g, float dt);
};

#endif
