/*
 * constants.h
 *
 *  Created on: Apr 29, 2025
 *      Author: pedroleal
 */

#ifndef SRC_CONSTANTS_CONSTANTS_H_
#define SRC_CONSTANTS_CONSTANTS_H_

#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#define micros() (HAL_GetTick() * 1000)
#define sq(x) ((x) * (x))
#define constrain(x, a, b) ((x) < (a) ? (a) : ((x) > (b) ? (b) : (x)))
#define TWO_PI 6.28318530718f

#include "main.h"

static inline float map(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

class Stm32Handlers {
public:
	SPI_HandleTypeDef *hspi1;
	UART_HandleTypeDef *huart3;
	UART_HandleTypeDef *huart4;
	UART_HandleTypeDef *huart5;
	TIM_HandleTypeDef *htim3, *htim4;
};

class Stm32HandlersSingleton
{
public:
    static Stm32Handlers *getInstance()
    {
        if (stm32h == nullptr)
        {
        	stm32h = new Stm32Handlers();
        }
        return stm32h;
    }

private:
    static Stm32Handlers *stm32h;
};

#endif /* SRC_CONSTANTS_CONSTANTS_H_ */
