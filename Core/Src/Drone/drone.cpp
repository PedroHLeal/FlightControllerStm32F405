/*
 * drone.cpp
 *
 *  Created on: Apr 27, 2025
 *      Author: pedroleal
 */

#include "drone.h"
#include <stdio.h>
#include "../Sensors/Gyro.h"
#include "../Sensors/mtp02p.h"
#include "../Sensors/battery.h"
#include "../Comms/ble.h"
#include "../Calcs/estimations.h"
#include "../motors.h"
#include "../pid.h"
#include "../timer.h"
#include "../globals.h"
#include "modes.h"

ControllerReadings *readings;
DronePosition p;
uint8_t rxuart3byte = 0x00, rxuart2byte = 0x00;
uint8_t rxuart4byte[5];
volatile uint32_t controlTicks = 0;   // written by the TIM2 ISR, polled by the loops

static void sendTelemetry() {
	TelemetryBLE t;
	t.armed = readings->armed != 0;
	t.flowAvailable = estimator.isPositioningAvailable;
	t.coasting = estimator.coasting;
	t.holding = p.holding;
	t.batteryValid = batteryValid();
	t.vbat = batteryVoltage();
	t.current = batteryCurrent();
	t.altitudeMm = estimator.positionZ;
	t.velX = estimator.estimatedVelX;
	t.velY = estimator.estimatedVelY;
	t.throttle = p.throttle;
	t.throttleI = p.throttleI;
	t.velXI = p.velXI;
	t.velYI = p.velYI;
	t.pitch = gyro.rotationAngle[0];
	t.roll = gyro.rotationAngle[1];
	t.velZ = estimator.velZ / 10.0f;
	t.tofMm = mtf02p.payload.distance;
	t.flowVelX = estimator.velX;
	t.flowVelY = estimator.velY;
	t.posX = estimator.estimatedPosX;
	t.posY = estimator.estimatedPosY;
	t.yaw = estimator.yawDeg;
	t.flowQuality = mtf02p.payload.flow_quality;
	sendTelemetryBLE(stm32Handlers.huart4, &t);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART3) {
		mtf02p.update(rxuart3byte);
		HAL_UART_Receive_IT(huart, &rxuart3byte, 1);  // Re-arm interrupt
	}

	if (huart->Instance == UART4) {
		getRemoteCommands(rxuart4byte);
		HAL_UART_Receive_IT(huart, rxuart4byte, 5);  // Re-arm interrupt
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {
		controlTicks++;
	}
}

#if MODE == DEBUG_MODE
void run() {
	gyro.accelInit();
	dwt_init();
	batteryInit();
	initControllerBLE();
	readings = getReadingsBLE();
	readings->armed = 0;
	readings->setPointPitch = 0;
	readings->setPointRoll = 0;
	readings->targetThrottle = 0;

	HAL_Delay(1000);
	while (!gyro.calibrate())
		;

	HAL_UART_Receive_IT(stm32Handlers.huart3, &rxuart3byte, 1);
	HAL_UART_Receive_IT(stm32Handlers.huart4, rxuart4byte, 5);

	p.usePositioning = false;
	p.useZPositioning = false;
	float dt = 0.001;
	long controlLoopUs = 0;
	HAL_TIM_Base_Start_IT(stm32Handlers.htim2);

	float elapsed = 0;
	uint32_t iterations = 0;

	while (1) {
		elapsed += dwt_dt_s(HAL_RCC_GetHCLKFreq());

		if (controlTicks) {
			controlTicks --;
			controlLoopUs += 1000;
			iterations++;
			gyro.updateData(dt, controlLoopUs);
			estimator.calculateEstimations(&gyro, &mtf02p, dt, controlLoopUs);
			runPositionHold(&p, &estimator, readings);
			runAngleModePid(&p, &gyro, &estimator, readings);
			batteryUpdate(dt);
			if (iterations % 100 == 0)
				sendTelemetry();
		}

		if (elapsed >= 0.02) {
			// Arduino Serial Plotter format: label:value pairs, one line per sample.
			// loopHz must read 1000: everything time-based assumes the loop keeps up.
			// fused vs flow velocity, the accel the fusion integrates, the estimator attitude and its gate
			printf("velX:%.1f,velY:%.1f,flowX:%.1f,flowY:%.1f,accX:%.0f,accY:%.0f,angU0:%.1f,angU1:%.1f,frozen:%d\n",
					estimator.estimatedVelX, estimator.estimatedVelY, estimator.velX, estimator.velY,
					estimator.accX, estimator.accY, gyro.rotationAngleUncal[0], gyro.rotationAngleUncal[1],
					gyro.uncalAccelFrozen ? 100 : 0);
			iterations = 0;
			elapsed = 0;
		}
	}
}
#endif

#if MODE == RUN_DRONE
void run() {
	gyro.accelInit();
	batteryInit();
	Motors *motors = MotorsSingleton::getInstance(stm32Handlers.htim3,
			stm32Handlers.htim4);

	initControllerBLE();
	readings = getReadingsBLE();
	readings->armed = 0;
	readings->setPointPitch = 0;
	readings->setPointRoll = 0;
	readings->targetThrottle = 0;

	HAL_Delay(1000);
	while (!gyro.calibrate())
		;

	motors->startup();

	HAL_UART_Receive_IT(stm32Handlers.huart3, &rxuart3byte, 1);
	HAL_UART_Receive_IT(stm32Handlers.huart4, rxuart4byte, 5);

	p.usePositioning = false;
	p.useZPositioning = false;
	float dt = 0.001;
	long controlLoopUs = 0;
	int telemetryTicks = 0;

	HAL_TIM_Base_Start_IT(stm32Handlers.htim2);

	while (1) {
		if (controlTicks) {
			controlTicks --;
			controlLoopUs += 1000;
			// battery and telemetry run disarmed too: the app shows the pack before takeoff
			// and the voltage filter is settled by the time the motors need the sag factor
			batteryUpdate(dt);
			if (++telemetryTicks >= 100) {
				telemetryTicks = 0;
				sendTelemetry();
			}
			// attitude runs disarmed too, so the telemetry angles are live on the ground
			gyro.updateData(dt, controlLoopUs);
			if (!readings->armed) {
				motors->writeAll(0);
				resetPid(&p);
				estimator.resetEstimations();
				// heading is gyro-only and the yaw loop holds it at zero: arming defines zero
				gyro.rotationAngle[2] = 0;
				gyro.rotationAngleUncal[2] = 0;
				continue;
			}
			estimator.calculateEstimations(&gyro, &mtf02p, dt, controlLoopUs);
			runPositionHold(&p, &estimator, readings);
			runAngleModePid(&p, &gyro, &estimator, readings);
			motors->writeDronePosition(&p);
		}
	}
}
#endif

#if MODE == CALIBRATE_OPTFLOW
void run() {
	gyro.accelInit();
	HAL_Delay(1000);
	while (!gyro.calibrate())
		;

	HAL_UART_Receive_IT(stm32Handlers.huart3, &rxuart3byte, 1);

	float dt = 0.001;
	long controlLoopUs = 0;
	uint32_t elapsedMs = 0;
	int lastPhase = -1;
	mtf02p.calibrationReset();
	HAL_TIM_Base_Start_IT(stm32Handlers.htim2);

	while (1) {
		if (!controlTicks)
			continue;
		controlTicks--;
		controlLoopUs += 1000;
		elapsedMs++;
		gyro.updateData(dt, controlLoopUs);

		int phase = elapsedMs < 5000 ? 0 :
					elapsedMs < 20000 ? 1 :
					elapsedMs < 25000 ? 2 :
					elapsedMs < 30000 ? 3 :
					elapsedMs < 35000 ? 4 :
					elapsedMs < 40000 ? 5 : 6;

		if (phase != lastPhase) {
			lastPhase = phase;
			switch (phase) {
			case 0: printf("Hold still and level, ~30 cm over a textured floor. Rotation phase in 5 s.\n"); break;
			case 1: printf("ROCK it in pitch AND roll for 15 s. No sliding.\n"); break;
			case 2: printf("Hold still. Forward push in 5 s.\n"); break;
			case 3: printf("Slide it FORWARD ~30 cm, then hold still.\n"); break;
			case 4: printf("Hold still. Right push in 5 s.\n"); break;
			case 5: printf("Slide it RIGHT ~30 cm, then hold still.\n"); break;
			case 6: mtf02p.calibrationReport(); break;
			}
		}

		switch (phase) {
		case 1: mtf02p.calibrationAccumulateRotation(&gyro); break;
		case 3: mtf02p.calibrationAccumulateForward(); break;
		case 5: mtf02p.calibrationAccumulateRight(); break;
		}

		if (phase < 6 && elapsedMs % 1000 == 0) {
			printf("  q=%d d=%lumm fx=%d fy=%d\n", mtf02p.payload.flow_quality,
					(unsigned long) mtf02p.payload.distance,
					mtf02p.payload.flow_vel_x, mtf02p.payload.flow_vel_y);
		}
		if (phase == 6 && elapsedMs % 5000 == 0)
			mtf02p.calibrationReport();
	}
}
#endif

#if MODE == TEST_MOTORS
void run() {
	Motors *motors = MotorsSingleton::getInstance(stm32Handlers.htim3,
			stm32Handlers.htim4);

	motors->startup();
	motors->testMotors();
}
#endif

