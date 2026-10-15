/*
 * BMI270.h
 *
 *  Created on: Apr 23, 2025
 *      Author: pedroleal
 */

#ifndef SRC_BMI270_BMI270_H_
#define SRC_BMI270_BMI270_H_

#include "main.h"
#include "../Calcs/DSPFilters/src/Filters.h"

// While the drone is accelerating horizontally the accelerometer cannot tell tilt from
// acceleration, so the estimator-path attitude runs on gyro alone until it calms down.
// Estimator-path attitude, Betaflight-style: the accel corrects the attitude only when its
// magnitude is near 1 g (the sensor's own test, independent of the attitude, so it cannot lock),
// and slowly, so transient accelerations average out instead of leaking into tilt. Fast gain
// for a few seconds after boot so the slow filter finds level quickly.
#define UNCAL_ACCEL_MIN_G 0.85f
#define UNCAL_ACCEL_MAX_G 1.15f
#define UNCAL_ACCEL_WEIGHT 0.0005f        // per 1 kHz tick: tau 2 s
#define UNCAL_ACCEL_WEIGHT_BOOT 0.02f     // tau 50 ms while converging after boot
#define UNCAL_BOOT_FAST_MS 3000

class Gyro {
public:
	float rawAccel[3] = { 0 };
	float accelCalibation[3] = {0};
	float accel[3] = { 0 };
	float rawGyro[3] = { 0 };
	float gyroCalibation[3] = {0};
	float gyro[3] = { 0 };
	float accelAngle[3] = { 0 };
	float rotationAngle[3] = { 0 };
	// Uncalibrated path: no boot offset subtracted, so the vector stays a rotation of gravity
	// and its own attitude estimate can cancel it. Feeds inertialAccel (estimators only);
	// the calibrated path above keeps "level" = boot attitude for the angle PIDs.
	float accelUncal[3] = { 0 };
	bool uncalAccelFrozen = false;   // true while the accel magnitude is outside the trusted band
	int uncalAgeMs = 0;
	float accelAngleUncal[3] = { 0 };
	float rotationAngleUncal[3] = { 0 };
	float inertialAccel[3] = { 0 };
	FilterOnePole *accelFilterX, *accelFilterY, *accelFilterZ;
	FilterOnePole *accelUncalFilterX, *accelUncalFilterY, *accelUncalFilterZ;
	FilterOnePole *gyroFilterX, *gyroFilterY, *gyroFilterZ;
	Gyro();
	uint8_t accelReadRegister(uint8_t reg);
	void accelWriteRegister(uint8_t reg, uint8_t data);
	void accelReadData();
	void updateData(float dt, long us);
	void processRawData(float dt, long us);
	void gyroDelayed(int delayMs, float *g0, float *g1, float *g2 = 0);
	void dumpData();
	bool calibrate();
	void accelInit();
private:
	// one entry per 1 kHz update, so index distance == milliseconds
	static const int GYRO_HISTORY = 256;
	float gyroHist[3][GYRO_HISTORY] = {};
	uint16_t gyroHistIdx = 0;
	int calibrationRounds = 3000, currentRound = 0;
	SPI_HandleTypeDef *hspi1;
	float accelCalibration[3] = { 0 };
};

class GyroSingleton
{
public:
    static Gyro *getInstance()
    {
        if (g == nullptr)
        {
            g = new Gyro();
        }
        return g;
    }

private:
    static Gyro *g;
};
#endif /* SRC_BMI270_BMI270_H_ */
