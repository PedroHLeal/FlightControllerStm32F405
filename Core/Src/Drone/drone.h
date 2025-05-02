/*
 * drone.h
 *
 *  Created on: Apr 27, 2025
 *      Author: pedroleal
 */

#ifndef SRC_DRONE_DRONE_H_
#define SRC_DRONE_DRONE_H_

#include "main.h"

void runDrone(SPI_HandleTypeDef *hspi1, UART_HandleTypeDef *huart3,
		UART_HandleTypeDef *huart6);

#endif /* SRC_DRONE_DRONE_H_ */
