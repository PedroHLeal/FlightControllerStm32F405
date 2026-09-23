/*
 * hc-06.h
 *
 *  Created on: May 2, 2025
 *      Author: pedroleal
 */

#ifndef SRC_COMMS_HC_06_H_
#define SRC_COMMS_HC_06_H_

#include "main.h"
#include "remote_controller.h"

void getRemoteCommands(uint8_t value);
ControllerReadings* getReadings();

#endif /* SRC_COMMS_HC_06_H_ */
