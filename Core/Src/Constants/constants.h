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

class Stm32Handlers {
public:
	SPI_HandleTypeDef *hspi1;
	UART_HandleTypeDef *huart3;
	UART_HandleTypeDef *huart6;
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
