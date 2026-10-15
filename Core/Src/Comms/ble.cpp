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

static int16_t clampI16(float v) {
	if (v > 32767.0f)
		return 32767;
	if (v < -32768.0f)
		return -32768;
	return (int16_t) v;
}

static int8_t clampI8(float v) {
	if (v > 127.0f)
		return 127;
	if (v < -128.0f)
		return -128;
	return (int8_t) v;
}

void sendTelemetryBLE(UART_HandleTypeDef *huart, const TelemetryBLE *t) {
	// Frame v2: A5 5A LEN payload... XOR, LEN = total bytes, XOR over bytes 2..LEN-2.
	// Fields are appended in order; a reader that knows fewer fields ignores the tail.
	static uint8_t frame[64];   // must outlive the interrupt-driven transmit
	if (HAL_UART_GetState(huart) != HAL_UART_STATE_READY
			&& HAL_UART_GetState(huart) != HAL_UART_STATE_BUSY_RX)
		return;   // previous frame still going out

	int n = 0;
	auto u8 = [&](uint8_t v) { frame[n++] = v; };
	auto i8s = [&](float v) { frame[n++] = (uint8_t) clampI8(v); };
	auto u16 = [&](uint16_t v) { frame[n++] = v & 0xFF; frame[n++] = v >> 8; };
	auto i16s = [&](float v) { int16_t x = clampI16(v); frame[n++] = x & 0xFF; frame[n++] = (uint16_t) x >> 8; };

	u8(0xA5);
	u8(0x5A);
	u8(0);   // length, filled below
	u8((t->armed ? 1 : 0) | (t->flowAvailable ? 2 : 0) | (t->coasting ? 4 : 0)
			| (t->holding ? 8 : 0) | (t->batteryValid ? 16 : 0));
	u16((uint16_t) (t->vbat * 100.0f));
	u16((uint16_t) (t->current * 100.0f));
	i16s(t->altitudeMm);
	i16s(t->velX * 10.0f);
	i16s(t->velY * 10.0f);
	u8((uint8_t) (t->throttle < 0 ? 0 : (t->throttle > 255 ? 255 : t->throttle)));
	i8s(t->throttleI);
	i8s(t->velXI * 10.0f);
	i8s(t->velYI * 10.0f);
	i16s(t->pitch * 10.0f);
	i16s(t->roll * 10.0f);
	i16s(t->velZ * 10.0f);
	u16((uint16_t) (t->tofMm < 0 ? 0 : (t->tofMm > 65535 ? 65535 : t->tofMm)));
	i16s(t->flowVelX * 10.0f);
	i16s(t->flowVelY * 10.0f);
	i16s(t->posX);
	i16s(t->posY);
	i16s(t->yaw * 10.0f);
	u8((uint8_t) (t->flowQuality < 0 ? 0 : (t->flowQuality > 255 ? 255 : t->flowQuality)));

	frame[2] = (uint8_t) (n + 1);
	uint8_t x = 0;
	for (int i = 2; i < n; i++)
		x ^= frame[i];
	frame[n++] = x;

	HAL_UART_Transmit_IT(huart, frame, n);
}
