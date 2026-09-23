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
	float inertialAccel[3] = { 0 };
	FilterOnePole *accelFilterX, *accelFilterY, *accelFilterZ;
	Gyro();
	uint8_t accelReadRegister(uint8_t reg);
	void accelWriteRegister(uint8_t reg, uint8_t data);
	void accelReadData();
	void updateData(float dt);
	void processRawData(float dt);
	void dumpData();
	bool calibrate();
	void accelInit();
private:
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
