/*
 * drone.cpp
 *
 *  Created on: Apr 27, 2025
 *      Author: pedroleal
 */

#include "drone.h"
#include <stdio.h>
#include "../Sensors/Gyro.h"
#include "../Sensors/mtp02p.h"
#include "../Comms/ble.h"
#include "../Calcs/estimations.h"
#include "../motors.h"
#include "../pid.h"
#include "../timer.h"

Stm32Handlers *Stm32HandlersSingleton::stm32h = nullptr;
MTF02P *mtf02 = MTF02PSingleton::getMTF02PInstance();
ControllerReadings *readings;
DronePosition p;
uint8_t rxuart3byte = 0x00, rxuart2byte = 0x00;
uint8_t rxuart4byte[5];

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART3) {
    	mtf02->update(rxuart3byte);
    	HAL_UART_Receive_IT(huart, &rxuart3byte, 1);  // Re-arm interrupt
	}

	if (huart->Instance == UART4) {
		getRemoteCommands(rxuart4byte);
		HAL_UART_Receive_IT(huart, rxuart4byte, 5);  // Re-arm interrupt
	}
}

void runDrone(Stm32Handlers *stm32Handlers) {
	Motors *motors = MotorsSingleton::getInstance(stm32Handlers->htim3, stm32Handlers->htim4);

	dwt_init();
	initControllerBLE();
	readings = getReadingsBLE();
	readings->armed = 0;

	Gyro *gyro = new Gyro();
	HAL_Delay(1000);
	while(!gyro->calibrate());

	motors->startup();
//	motors->testMotors();

	Estimator *e = new Estimator();

	HAL_UART_Receive_IT(stm32Handlers->huart3, &rxuart3byte, 1);
	HAL_UART_Receive_IT(stm32Handlers->huart4, rxuart4byte, 5);

	p.usePositioning = false;
	p.useZPositioning = false;

	while (1) {
		float dt = dwt_dt_s(HAL_RCC_GetHCLKFreq());
		if (!readings->armed) {
			motors->writeAll(0);
			resetPid(&p);
			continue;
		}

		gyro->updateData(dt);
		e->calculateEstimations(gyro, mtf02, dt);
		calculatePidThrottle(&p, readings, e);
		calculatePidPitch(&p, readings, gyro);
		calculatePidRoll(&p, readings, gyro);
		calculatePidPitchRate(&p, readings, gyro);
		calculatePidRollRate(&p, readings, gyro);
		calculatePidYaw(&p, gyro);
		motors->writeDronePosition(&p);
	}
}

