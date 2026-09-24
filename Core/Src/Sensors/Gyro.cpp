/*
 * BMI270.c
 *
 *  Created on: Apr 23, 2025
 *      Author: pedroleal
 */

#include <math.h>
#include "../Sensors/Gyro.h"
#include "../Constants/constants.h"
#include "../globals.h"
#include <stdio.h>

#define LP_FILTER

Gyro::Gyro() {
	accelFilterX = new FilterOnePole(LOWPASS, 15);
	accelFilterY = new FilterOnePole(LOWPASS, 15);
	accelFilterZ = new FilterOnePole(LOWPASS, 15);

	accelUncalFilterX = new FilterOnePole(LOWPASS, 15);
	accelUncalFilterY = new FilterOnePole(LOWPASS, 15);
	accelUncalFilterZ = new FilterOnePole(LOWPASS, 15);

	gyroFilterX = new FilterOnePole(LOWPASS, 90);
	gyroFilterY = new FilterOnePole(LOWPASS, 90);
	gyroFilterZ = new FilterOnePole(LOWPASS, 90);
}

void Gyro::accelInit() {
	this->hspi1 = stm32Handlers.hspi1;

	// Reset device and wait
	accelWriteRegister(0x11, 0x01);  // DEVICE_CONFIG
	HAL_Delay(100);

	accelWriteRegister(0x4E, 0x0F);
	accelWriteRegister(0x4F, 0x66); // gyro config
	accelWriteRegister(0x50, 0x26); // accel config

#ifdef LP_FILTER
	const uint8_t ACCEL_UI_BW_CODE = 7;   // lowest (tightest) BW option from the table
	const uint8_t GYRO_UI_BW_CODE  = 2;   // keep your gyro BW for now (can tune later)

	const uint8_t ACCEL_UI_ORD = 0b10;    // 3rd order (stronger attenuation)
	const uint8_t GYRO_UI_ORD  = 0b01;    // 2nd order (reasonable compromise)

	uint8_t ui_bw = accelReadRegister(0x52);

	ui_bw = (ui_bw & ~0x0F) | (GYRO_UI_BW_CODE & 0x0F);

	ui_bw = (ui_bw & ~0xF0) | ((ACCEL_UI_BW_CODE & 0x0F) << 4);

	accelWriteRegister(0x52, ui_bw);

	uint8_t gyro_ord = accelReadRegister(0x51);
	gyro_ord = (gyro_ord & ~0x0C) | ((GYRO_UI_ORD & 0x03) << 2);
	accelWriteRegister(0x51, gyro_ord);

	uint8_t accel_ord = accelReadRegister(0x53);
	accel_ord = (accel_ord & ~0x18) | ((ACCEL_UI_ORD & 0x03) << 3);
	accelWriteRegister(0x53, accel_ord);
#endif
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
		rawAccel[i] = (int16_t) ((rx[2 * i] << 8) | rx[2 * i + 1]) / 4096.0f;
		rawGyro[i] = (int16_t) ((rx[6 + 2 * i] << 8) | rx[6 + 2 * i + 1])
				/ 131.0f;
	}
}

void Gyro::updateData(float dt, long us) {
	this->accelReadData();
	this->processRawData(dt, us);
}

void Gyro::processRawData(float dt, long us) {
	accel[0] = rawAccel[0] - accelCalibation[0];
	accel[1] = rawAccel[1] - accelCalibation[1];
	accel[2] = rawAccel[2];
	gyro[0] = rawGyro[0] - gyroCalibation[0];
	gyro[1] = rawGyro[1] - gyroCalibation[1];
	gyro[2] = rawGyro[2] - gyroCalibation[2];

	accelFilterX->input(accel[0], us);
	accel[0] = accelFilterX->output();

	accelFilterY->input(accel[1], us);
	accel[1] = accelFilterY->output();

	accelFilterZ->input(accel[2], us);
	accel[2] = accelFilterZ->output();

	accelUncalFilterX->input(rawAccel[0], us);
	accelUncal[0] = accelUncalFilterX->output();

	accelUncalFilterY->input(rawAccel[1], us);
	accelUncal[1] = accelUncalFilterY->output();

	accelUncalFilterZ->input(rawAccel[2], us);
	accelUncal[2] = accelUncalFilterZ->output();

	gyroFilterX->input(gyro[0], us);
	gyro[0] = gyroFilterX->output();

	gyroFilterY->input(gyro[1], us);
	gyro[1] = gyroFilterY->output();

	gyroFilterZ->input(gyro[2], us);
	gyro[2] = gyroFilterZ->output();

	gyroHist[0][gyroHistIdx] = gyro[0];
	gyroHist[1][gyroHistIdx] = gyro[1];
	gyroHistIdx = (gyroHistIdx + 1) & (GYRO_HISTORY - 1);

//	printf("%.3f %.3f %.3f %d %d\n", accel[0],
//			accel[1], accel[2], 1, -1);

//	printf("%.3f %.3f %.3f %d %d\n", gyro[0],
//				gyro[1], gyro[2], 30, -30);

	// Detect accel angle
	accelAngle[0] = (atanf(
			accel[1] / sqrtf(accel[0] * accel[0] + accel[2] * accel[2])))
			* RAD_TO_DEG; // ANGLEROLL
	accelAngle[1] = (-atanf(
			accel[0] / sqrtf(accel[1] * accel[1] + accel[2] * accel[2])))
			* RAD_TO_DEG; // ANGLEPITCH
	accelAngle[2] = (atan2f(accel[1], accel[2])) * RAD_TO_DEG;

	rotationAngle[0] = 0.998f * (rotationAngle[0] + gyro[0] * dt)
			+ 0.002f * accelAngle[0];
	rotationAngle[1] = 0.998f * (rotationAngle[1] + gyro[1] * dt)
			+ 0.002f * accelAngle[1];
	rotationAngle[2] += gyro[2] * dt;

	// Same estimate on the uncalibrated vector, referenced to true gravity
	accelAngleUncal[0] = (atanf(
			accelUncal[1] / sqrtf(accelUncal[0] * accelUncal[0] + accelUncal[2] * accelUncal[2])))
			* RAD_TO_DEG;
	accelAngleUncal[1] = (-atanf(
			accelUncal[0] / sqrtf(accelUncal[1] * accelUncal[1] + accelUncal[2] * accelUncal[2])))
			* RAD_TO_DEG;

	float horizontalG = sqrtf(inertialAccel[0] * inertialAccel[0] + inertialAccel[1] * inertialAccel[1]);
	uncalAccelFrozen = horizontalG > UNCAL_ACCEL_GATE_G && uncalFreezeMs < UNCAL_ACCEL_GATE_MAX_MS;
	if (uncalAccelFrozen) {
		uncalFreezeMs++;
		rotationAngleUncal[0] += gyro[0] * dt;
		rotationAngleUncal[1] += gyro[1] * dt;
	} else {
		uncalFreezeMs = 0;
		rotationAngleUncal[0] = 0.998f * (rotationAngleUncal[0] + gyro[0] * dt)
				+ 0.002f * accelAngleUncal[0];
		rotationAngleUncal[1] = 0.998f * (rotationAngleUncal[1] + gyro[1] * dt)
				+ 0.002f * accelAngleUncal[1];
	}

	float sr = sinf(rotationAngleUncal[0] * DEG_TO_RAD), cr = cosf(rotationAngleUncal[0] * DEG_TO_RAD);
	float sp = sinf(rotationAngleUncal[1] * DEG_TO_RAD), cp = cosf(rotationAngleUncal[1] * DEG_TO_RAD);

	inertialAccel[0] = accelUncal[0] * cp
			+ accelUncal[1] * sr * sp
			+ accelUncal[2] * sp * cr;
	inertialAccel[1] = accelUncal[0] * sr * sp
			+ accelUncal[1] * cr
			- accelUncal[2] * sr * cp;
	inertialAccel[2] = -accelUncal[0] * sp
			+ accelUncal[1] * cp * sr
			+ accelUncal[2] * cp * cr;
}

void Gyro::gyroDelayed(int delayMs, float *g0, float *g1) {
	if (delayMs < 0)
		delayMs = 0;
	if (delayMs > GYRO_HISTORY - 1)
		delayMs = GYRO_HISTORY - 1;
	int idx = (gyroHistIdx - 1 - delayMs) & (GYRO_HISTORY - 1);
	*g0 = gyroHist[0][idx];
	*g1 = gyroHist[1][idx];
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
