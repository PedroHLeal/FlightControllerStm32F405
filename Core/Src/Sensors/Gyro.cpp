/*
 * BMI270.c
 *
 *  Created on: Apr 23, 2025
 *      Author: pedroleal
 */
#include <math.h>
#include "../Sensors/Gyro.h"
#include "../Constants/constants.h"

Gyro::Gyro() {
	Stm32Handlers* stm32h = Stm32HandlersSingleton::getInstance();
	this->hspi1 = stm32h->hspi1;
	this->accelInit();
}

void Gyro::accelInit() {
	// Wake up the sensor by clearing sleep bit (6B = 0x00)
	uint8_t tx[2];

	tx[0] = 0x6B; // PWR_MGMT_1 register
	tx[1] = 0x00; // Set to 0 to wake up

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi1, tx, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	HAL_Delay(100); // Give some time after wakeup

	accelWriteRegister(0x1A, 0x05);
	accelWriteRegister(0x1C, 0x10);
	accelWriteRegister(0x3B, 0x00);
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
	uint8_t tx[2] = { (uint8_t)(reg & 0x7F), data };
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi1, tx, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

void Gyro::accelReadData() {
	uint8_t tx = 0x3B | 0x80;
	uint8_t rx[14];

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

	HAL_SPI_Transmit(hspi1, &tx, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(hspi1, rx, 14, HAL_MAX_DELAY);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	rawAccel[0] = ((int16_t) (rx[0] << 8 | rx[1])) / 4096.0;
	rawAccel[1] = ((int16_t) (rx[2] << 8 | rx[3])) / 4096.0;
	rawAccel[2] = ((int16_t) (rx[4] << 8 | rx[5])) / 4096.0;

	rawGyro[0] = ((int16_t) (rx[8] << 8 | rx[9])) / 131.0;
	rawGyro[1] = ((int16_t) (rx[10] << 8 | rx[11])) / 131.0;
	rawGyro[2] = ((int16_t) (rx[12] << 8 | rx[13])) / 131.0;
}

void Gyro::updateData(float dt) {
	this->accelReadData();
	this->processRawData(dt);
}

void Gyro::processRawData(float dt) {
	accel[0] = rawAccel[0] - accelCalibation[0];
	accel[1] = rawAccel[1] - accelCalibation[1];
	accel[2] = rawAccel[2] - accelCalibation[2];
	gyro[0] = rawGyro[0] - gyroCalibation[0];
	gyro[1] = rawGyro[1] - gyroCalibation[1];
	gyro[2] = rawGyro[2] - gyroCalibation[2];

	// Detect accel angle
	accelAngle[0] = (atan(accel[1] / sqrt(accel[0]*accel[0] + accel[2] * accel[2]))) * RAD_TO_DEG; // ANGLEROLL
	accelAngle[1] = (-atan(accel[0] / sqrt(accel[1]*accel[1] + accel[2]*accel[2]))) * RAD_TO_DEG; // ANGLEPITCH
	accelAngle[2] = (atan2(accel[1], accel[2])) * RAD_TO_DEG;

	rotationAngle[0] = 0.98 * (rotationAngle[0] + gyro[0] * dt) + 0.02 * accelAngle[0];
	rotationAngle[1] = 0.98 * (rotationAngle[1] + gyro[1] * dt) + 0.02 * accelAngle[1];
	rotationAngle[2] += gyro[2] * dt;

	inertialAccel[0] = accel[0] * cos(rotationAngle[1] * DEG_TO_RAD) + accel[1] * sin(rotationAngle[0]  * DEG_TO_RAD) * sin(rotationAngle[1] * DEG_TO_RAD) + accel[2] * sin(rotationAngle[1] * DEG_TO_RAD) * cos(rotationAngle[0] * DEG_TO_RAD);
	inertialAccel[1] = accel[0] * sin(rotationAngle[0] * DEG_TO_RAD) * sin(rotationAngle[1] * DEG_TO_RAD) + accel[1] * cos(rotationAngle[0] * DEG_TO_RAD) - accel[2] * sin(rotationAngle[0] * DEG_TO_RAD) * cos(rotationAngle[1] * DEG_TO_RAD);
	inertialAccel[2] = -accel[0] * sin(rotationAngle[1] * DEG_TO_RAD) + accel[1] * cos(rotationAngle[1] * DEG_TO_RAD) * sin(rotationAngle[0] * DEG_TO_RAD) + accel[2] * cos(rotationAngle[1] * DEG_TO_RAD) * cos(rotationAngle[0] * DEG_TO_RAD);
}
