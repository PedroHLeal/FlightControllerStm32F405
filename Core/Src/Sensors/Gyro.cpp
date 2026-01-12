/*
 * BMI270.c
 *
 *  Created on: Apr 23, 2025
 *      Author: pedroleal
 */
#include <math.h>
#include "../Sensors/Gyro.h"
#include "../Constants/constants.h"
#include <stdio.h>

Gyro::Gyro() {
	Stm32Handlers *stm32h = Stm32HandlersSingleton::getInstance();
	this->hspi1 = stm32h->hspi1;
	this->accelInit();
}

void Gyro::accelInit() {
	// Reset device and wait
	accelWriteRegister(0x11, 0x01);  // DEVICE_CONFIG
	HAL_Delay(100);

	accelWriteRegister(0x4E, 0x0F);
	accelWriteRegister(0x4F, 0x66); // gyro config
	accelWriteRegister(0x50, 0x26); // accel config
}

uint8_t Gyro::accelReadRegister(uint8_t reg) {
	uint8_t tx, rx;

	tx = reg | 0x80;

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi1, &tx, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(hspi1, &rx, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	return rx;
}

void Gyro::accelWriteRegister(uint8_t reg, uint8_t data) {
	uint8_t tx[2] = { (uint8_t) (reg & 0x7F), data };
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi1, tx, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

void Gyro::accelReadData() {
	uint8_t tx = 0x1F | 0x80;
	uint8_t rx[12];

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

	HAL_SPI_Transmit(hspi1, &tx, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(hspi1, rx, 12, HAL_MAX_DELAY);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	for (int i = 0; i < 3; i++) {
		rawAccel[i] = (int16_t) ((rx[2 * i] << 8) | rx[2 * i + 1]) / 4096.0;
		rawGyro[i] = (int16_t) ((rx[6 + 2 * i] << 8) | rx[6 + 2 * i + 1])
				/ 131.0;
	}
}

void Gyro::updateData(float dt) {
	this->accelReadData();
	this->processRawData(dt);
}

void Gyro::processRawData(float dt) {
	accel[0] = rawAccel[0] - accelCalibation[0];
	accel[1] = rawAccel[1] - accelCalibation[1];
	accel[2] = rawAccel[2];
	gyro[0] = rawGyro[0] - gyroCalibation[0];
	gyro[1] = rawGyro[1] - gyroCalibation[1];
	gyro[2] = rawGyro[2] - gyroCalibation[2];

//	printf("%.3f %.3f %.3f %d %d\n", accel[0],
//			accel[1], accel[2], 1, -1);

//	printf("%.3f %.3f %.3f %d %d\n", gyro[0],
//				gyro[1], gyro[2], 30, -30);

	// Detect accel angle
	accelAngle[0] = (atan(
			accel[1] / sqrt(accel[0] * accel[0] + accel[2] * accel[2])))
			* RAD_TO_DEG; // ANGLEROLL
	accelAngle[1] = (-atan(
			accel[0] / sqrt(accel[1] * accel[1] + accel[2] * accel[2])))
			* RAD_TO_DEG; // ANGLEPITCH
	accelAngle[2] = (atan2(accel[1], accel[2])) * RAD_TO_DEG;

	rotationAngle[0] = 0.98 * (rotationAngle[0] + gyro[0] * dt)
			+ 0.02 * accelAngle[0];
	rotationAngle[1] = 0.98 * (rotationAngle[1] + gyro[1] * dt)
			+ 0.02 * accelAngle[1];
	rotationAngle[2] += gyro[2] * dt;

	inertialAccel[0] = accel[0] * cos(rotationAngle[1] * DEG_TO_RAD)
			+ accel[1] * sin(rotationAngle[0] * DEG_TO_RAD)
					* sin(rotationAngle[1] * DEG_TO_RAD)
			+ accel[2] * sin(rotationAngle[1] * DEG_TO_RAD)
					* cos(rotationAngle[0] * DEG_TO_RAD);
	inertialAccel[1] = accel[0] * sin(rotationAngle[0] * DEG_TO_RAD)
			* sin(rotationAngle[1] * DEG_TO_RAD)
			+ accel[1] * cos(rotationAngle[0] * DEG_TO_RAD)
			- accel[2] * sin(rotationAngle[0] * DEG_TO_RAD)
					* cos(rotationAngle[1] * DEG_TO_RAD);
	inertialAccel[2] = -accel[0] * sin(rotationAngle[1] * DEG_TO_RAD)
			+ accel[1] * cos(rotationAngle[1] * DEG_TO_RAD)
					* sin(rotationAngle[0] * DEG_TO_RAD)
			+ accel[2] * cos(rotationAngle[1] * DEG_TO_RAD)
					* cos(rotationAngle[0] * DEG_TO_RAD);
}

bool Gyro::calibrate() {
	this->accelReadData();
	if (this->currentRound < this->calibrationRounds) {
		accelCalibation[0] += rawAccel[0];
		accelCalibation[1] += rawAccel[1];
		accelCalibation[2] += rawAccel[2];
		gyroCalibation[0] += rawGyro[0];
		gyroCalibation[1] += rawGyro[1];
		gyroCalibation[2] += rawGyro[2];

		this->currentRound++;
		return false;
	} else {
		accelCalibation[0] /= calibrationRounds;
		accelCalibation[1] /= calibrationRounds;
		accelCalibation[2] /= calibrationRounds;

		gyroCalibation[0] /= calibrationRounds;
		gyroCalibation[1] /= calibrationRounds;
		gyroCalibation[2] /= calibrationRounds;

//		printf("%.3f %.3f %.3f %.3f %.3f %.3f\n", accelCalibation[0],
//				accelCalibation[1], accelCalibation[2], gyroCalibation[0],
//				gyroCalibation[1], gyroCalibation[2]);
		return true;
	}

}

void Gyro::dumpData() {
	printf("%.2f %.2f %.2f %.2f %.2f %.2f\n", rawAccel[0], rawAccel[1],
			rawAccel[2], rawGyro[0], rawGyro[1], rawGyro[2]);
}
