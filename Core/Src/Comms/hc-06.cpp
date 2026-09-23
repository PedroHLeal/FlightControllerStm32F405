/*
 * hc-06.cpp
 *
 *  Created on: May 2, 2025
 *      Author: pedroleal
 */

#include <stdlib.h>
#include <stdio.h>
#include "hc-06.h"

char btReading[20];
int btCurrent = 0;
int readValues[4];

ControllerReadings *r = (ControllerReadings*) malloc(
		sizeof(ControllerReadings));

void getRemoteCommands(uint8_t value) {
	btReading[btCurrent] = (char)value;
	btCurrent++;
	if (btReading[btCurrent - 1] == '\n') {
		int c = 0;
		char value[6];

		// reading throttle
		int v = 0;
		while (c < btCurrent) {
			if (btReading[c] != ' ') {
				value[v] = btReading[c];
			} else {
				value[v] = '\n';
				break;
			}
			v++;
			c++;
		}
		r->targetThrottle = atoi(value);

		// reading pitch
		v = 0;
		c++;
		while (c < btCurrent) {
			if (btReading[c] != ' ') {
				value[v] = btReading[c];
			} else {
				value[v] = '\n';
				break;
			}
			v++;
			c++;
		}
		r->setPointPitch = atoi(value) * 2;

		// reading roll
		v = 0;
		c++;
		while (c < btCurrent) {
			if (btReading[c] != ' ') {
				value[v] = btReading[c];
			} else {
				value[v] = '\n';
				break;
			}
			v++;
			c++;
		}
		r->setPointRoll = atoi(value) * 2;
		// Serial.println(String(value).toInt());

		// reading ARM
		v = 0;
		c++;
		while (c < btCurrent) {
			if (btReading[c] != ' ') {
				value[v] = btReading[c];
			} else {
				value[v] = '\n';
				break;
			}
			v++;
			c++;
		}
		r->armed = atoi(value) == 1800;

		// r->desiredHeight += r->targetThrottle * 0.5;

		// Serial.println(String(r->targetThrottle) + " " + String(r->setPointPitch) + " " + String(r->setPointRoll) + " " + String(r->armed));

		btCurrent = 0;
	}
}

ControllerReadings* getReadings() {
	return r;
}
