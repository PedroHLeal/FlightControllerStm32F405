/*
 * ble.cpp
 *
 *  Created on: Dec 16, 2025
 *      Author: pedroleal
 */

#include <stdio.h>
#include <stdlib.h>
#include "../Constants/constants.h"
#include "ble.h"

ControllerReadings *readingsBLE;

void getRemoteCommands(uint8_t *values) {
	readingsBLE->armed = values[0];
	readingsBLE->targetThrottle = values[1] - 100;
	readingsBLE->setPointRoll = constrain(-(values[2] - 30) * 3, -25, 25);
	readingsBLE->setPointPitch = constrain((values[3] - 30) * 3, -25, 25);
}

void initControllerBLE() {
	readingsBLE = (ControllerReadings*) malloc(sizeof(ControllerReadings));
}

ControllerReadings* getReadingsBLE() {
	return readingsBLE;
}
