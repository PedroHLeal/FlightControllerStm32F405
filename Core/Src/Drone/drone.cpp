/*
 * drone.cpp
 *
 *  Created on: Apr 27, 2025
 *      Author: pedroleal
 */

#include "drone.h"
#include <stdio.h>
#include "../Constants/constants.h"
#include "../Sensors/Gyro.h"
#include "../Sensors/mtp02p.h"
#include "../Comms/hc-06.h"

Stm32Handlers *Stm32HandlersSingleton::stm32h = nullptr;
MTF02P *mtf02 = MTF02PSingleton::getMTF02PInstance();

uint8_t rxuart3byte = 0x00, rxuart6byte = 0x00;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

    if (huart->Instance == USART3) {
    	getRemoteCommands(rxuart3byte);
    	ControllerReadings *r = getReadings();
    	printf("%f %f %f %d\n", r->targetThrottle, r->setPointPitch, r->setPointRoll, r->armed);
		HAL_UART_Receive_IT(huart, &rxuart3byte, 1);  // Re-arm interrupt
    }

    if (huart->Instance == USART6) {
    	printf("%02X\n", rxuart6byte);
//    	mtf02->update(rxuart6byte);
    	HAL_UART_Receive_IT(huart, &rxuart6byte, 1);  // Re-arm interrupt
	}

}

void runDrone(SPI_HandleTypeDef *hspi1, UART_HandleTypeDef *huart3,
		UART_HandleTypeDef *huart6) {
	uint32_t time_start = 0;

	// THIS SET A CLASS TO CONTAIN ALL STM32 HANDLERS TO BE ACCESSED GLOBALLY
	Stm32Handlers* stm32h = Stm32HandlersSingleton::getInstance();
	stm32h->hspi1 = hspi1;
	stm32h->huart3 = huart3;
	stm32h->huart6 = huart6;

	Gyro *gyro = new Gyro();

	HAL_UART_Receive_IT(stm32h->huart3, &rxuart3byte, 1);
	HAL_UART_Receive_IT(stm32h->huart6, &rxuart6byte, 1);

	while (gyro->accelReadRegister(0x75) != 0xAF)
		;
	while (1) {
		float dt = (HAL_GetTick() - time_start) / 1000.0;
		gyro->updateData(dt);
		time_start = HAL_GetTick();
		HAL_Delay(10);
	}
}
