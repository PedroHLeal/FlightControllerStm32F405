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
#include "../globals.h"
#include "modes.h"

ControllerReadings *readings;
DronePosition p;
uint8_t rxuart3byte = 0x00, rxuart2byte = 0x00;
uint8_t rxuart4byte[5];
uint32_t controlTicks = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART3) {
		mtf02p.update(rxuart3byte);
		HAL_UART_Receive_IT(huart, &rxuart3byte, 1);  // Re-arm interrupt
	}

	if (huart->Instance == UART4) {
		getRemoteCommands(rxuart4byte);
		HAL_UART_Receive_IT(huart, rxuart4byte, 5);  // Re-arm interrupt
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {
		controlTicks++;
	}
}

#if MODE == DEBUG_MODE
void run() {
	gyro.accelInit();
	dwt_init();
	initControllerBLE();
	readings = getReadingsBLE();
	readings->armed = 0;
	readings->setPointPitch = 0;
	readings->setPointRoll = 0;
	readings->targetThrottle = 0;

	HAL_Delay(1000);
	while (!gyro.calibrate())
		;

	HAL_UART_Receive_IT(stm32Handlers.huart3, &rxuart3byte, 1);
	HAL_UART_Receive_IT(stm32Handlers.huart4, rxuart4byte, 5);

	p.usePositioning = false;
	p.useZPositioning = false;
	float dt = 0.001;
	HAL_TIM_Base_Start_IT(stm32Handlers.htim2);

	float elapsed = 0;

	while (1) {
		elapsed += dwt_dt_s(HAL_RCC_GetHCLKFreq());

		if (controlTicks) {
			controlTicks --;
			gyro.updateData(dt);
			estimator.calculateEstimations(&gyro, &mtf02p, dt);
			runAngleModePid(&p, &gyro, &estimator, readings);
		}

		if (elapsed >= 0.01) {
			printf("%.2f\n", readings->setPointPitch);
			elapsed = 0;
		}
	}
}
#endif

#if MODE == RUN_DRONE
void run() {
	gyro.accelInit();
	Motors *motors = MotorsSingleton::getInstance(stm32Handlers.htim3,
			stm32Handlers.htim4);

	initControllerBLE();
	readings = getReadingsBLE();
	readings->armed = 0;
	readings->setPointPitch = 0;
	readings->setPointRoll = 0;
	readings->targetThrottle = 0;

	HAL_Delay(1000);
	while (!gyro.calibrate())
		;

	motors->startup();

	HAL_UART_Receive_IT(stm32Handlers.huart3, &rxuart3byte, 1);
	HAL_UART_Receive_IT(stm32Handlers.huart4, rxuart4byte, 5);

	p.usePositioning = false;
	p.useZPositioning = false;
	float dt = 0.001;

	HAL_TIM_Base_Start_IT(stm32Handlers.htim2);

	while (1) {
		if (controlTicks) {
			controlTicks --;
			if (!readings->armed) {
				motors->writeAll(0);
				resetPid(&p);
				estimator.resetEstimations();
				continue;
			}
			gyro.updateData(dt);
			estimator.calculateEstimations(&gyro, &mtf02p, dt);
			runAngleModePid(&p, &gyro, &estimator, readings);
			motors->writeDronePosition(&p);
		}
	}
}
#endif

#if MODE == CALIBRATE_OPTFLOW_X
void run() {
	float dt = 0.001;
	while(1) {
		gyro.updateData(dt);
		mtf02p.calibrateX(gyro.gyro[0], gyro.gyro[1]);
	}
}
#endif


#if MODE == CALIBRATE_OPTFLOW_Y
void run() {
	float dt = 0.001;
	while(1) {
		gyro.updateData(dt);
		mtf02p.calibrateY(gyro.gyro[0], gyro.gyro[1]);
	}
}
#endif

#if MODE == TEST_MOTORS
void run() {
	Motors *motors = MotorsSingleton::getInstance(stm32Handlers.htim3,
			stm32Handlers.htim4);

	motors->startup();
	motors->testMotors();
}
#endif

