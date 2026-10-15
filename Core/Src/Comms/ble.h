/*
 * ble.h
 *
 *  Created on: Dec 16, 2025
 *      Author: pedroleal
 */

#ifndef SRC_COMMS_BLE_H_
#define SRC_COMMS_BLE_H_

#include "main.h"
#include "remote_controller.h"

void initControllerBLE();
void getRemoteCommands(uint8_t* values);
ControllerReadings* getReadingsBLE();

// Drone -> phone, 18 bytes, little-endian: A5 5A flags vbat*100(u16) current*100(u16)
// altitude_mm(i16) velX*10(i16) velY*10(i16) throttle(u8) throttleI(i8) velXI*10(i8) velYI*10(i8)
// xor(bytes 2..16)
typedef struct {
    bool armed, flowAvailable, coasting, holding, batteryValid;
    float vbat, current, altitudeMm, velX, velY, throttle, throttleI, velXI, velYI;
    float pitch, roll;   // PID-path attitude, deg
    float velZ;          // cm/s, estimator
    float tofMm;         // raw ToF distance
    float flowVelX, flowVelY;   // cm/s, flow-only velocity before fusion
    float posX, posY;           // cm, arm-heading frame
    float yaw;                  // deg, gyro heading since arm
    float flowQuality;          // sensor quality 0..255, FLOW_MIN_QUALITY gates it
} TelemetryBLE;

void sendTelemetryBLE(UART_HandleTypeDef *huart, const TelemetryBLE *t);

#endif /* SRC_COMMS_BLE_H_ */
