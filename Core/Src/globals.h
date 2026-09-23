/*
* globals.h
*
*  Created on: Jan 31, 2026
*      Author: pedroleal
*/

#ifndef SRC_GLOBALS_H_
#define SRC_GLOBALS_H_

#include "main.h"
// #include "Constants/constants.h"
#include "Sensors/Gyro.h"
#include "Sensors/mtp02p.h"
#include "Calcs/estimations.h"

class Stm32Handlers {
public:
	SPI_HandleTypeDef *hspi1;
	UART_HandleTypeDef *huart3;
	UART_HandleTypeDef *huart4;
	UART_HandleTypeDef *huart5;
	TIM_HandleTypeDef *htim2, *htim3, *htim4;
};

extern Stm32Handlers stm32Handlers;
extern Gyro gyro;
extern MTF02P mtf02p;
extern Estimator estimator;

#endif /* SRC_GLOBALS_H_ */
