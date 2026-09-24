/*
 * constants.h
 *
 *  Created on: Apr 29, 2025
 *      Author: pedroleal
 */

#ifndef SRC_CONSTANTS_CONSTANTS_H_
#define SRC_CONSTANTS_CONSTANTS_H_

#define DEG_TO_RAD 0.017453292519943295f
#define RAD_TO_DEG 57.29577951308232f
#define micros() (HAL_GetTick() * 1000)
#define sq(x) ((x) * (x))
#define constrain(x, a, b) ((x) < (a) ? (a) : ((x) > (b) ? (b) : (x)))
#define TWO_PI 6.28318530718f

#include "main.h"

static inline float map(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#endif /* SRC_CONSTANTS_CONSTANTS_H_ */
