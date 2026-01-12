/*
 * ble.cpp
 *
 *  Created on: Dec 16, 2025
 *      Author: pedroleal
 */

#include <stdio.h>
#include <stdlib.h>
#include "ble.h"

ControllerReadings *readingsBLE;

void getRemoteCommands(uint8_t *values) {
	readingsBLE->armed = values[0];
	readingsBLE->targetThrottle = values[1] - 100;
	readingsBLE->setPointRoll = values[2] - 7;
	readingsBLE->setPointPitch = values[3] - 7;
}

void initControllerBLE() {
	readingsBLE = (ControllerReadings*) malloc(sizeof(ControllerReadings));
}

ControllerReadings* getReadingsBLE() {
	return readingsBLE;
}
