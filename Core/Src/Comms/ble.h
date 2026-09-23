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

#endif /* SRC_COMMS_BLE_H_ */
