/*
 * drone.cpp
 *
 *  Created on: Apr 27, 2025
 *      Author: pedroleal
 */

#include "drone.h"
#include <stdio.h>

#include "../Sensors/Gyro.h"

void runDrone(SPI_HandleTypeDef *hspi1) {
	uint32_t time_start = 0;
	Gyro *gyro = new Gyro(hspi1);

	while (gyro->accelReadRegister(0x75) != 0xAF)
		;
	while (1) {
		float dt = (HAL_GetTick() - time_start) / 1000.0;
		gyro->updateData(dt);
		time_start = HAL_GetTick();
		printf("%.2f %.2f %.2f\n", gyro->inertialAccel[0], gyro->inertialAccel[1], gyro->inertialAccel[2]);
		HAL_Delay(10);
	}
}
