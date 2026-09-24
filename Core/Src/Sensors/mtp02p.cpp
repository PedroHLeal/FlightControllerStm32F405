/*
 * mtp02p.cpp
 *
 *  Created on: Apr 29, 2025
 *      Author: pedroleal
 */

#include "mtp02p.h"
#include "../Constants/constants.h"
#include <stdio.h>
#include <math.h>

bool MTF02P::micolink_decode(uint8_t data) {
	if (micolink_parse_char(&msg, data) == false) {
		return false;
	}

	switch (msg.msg_id) {
	case MICOLINK_MSG_ID_RANGE_SENSOR: {
		memcpy(&payload, msg.payload, msg.len);
		/*
		 You can get the sensor data here:

		 distance           = payload.distance;
		 distance strength  = payload.strength;
		 distance precision = payload.precision;
		 distance status    = payload.tof_status;
		 flow velocity x    = payload.flow_vel_x;
		 flow velocity y    = payload.flow_vel_y;
		 flow quality       = payload.flow_quality;
		 flow status        = payload.flow_status;
		 */
		break;
	}

	default:
		break;
	}

	return true;
}

bool MTF02P::micolink_check_sum(MICOLINK_MSG_t *msg) {
	uint8_t length = msg->len + 6;
	uint8_t temp[MICOLINK_MAX_LEN];
	uint8_t checksum = 0;

	memcpy(temp, msg, length);

	for (uint8_t i = 0; i < length; i++) {
		checksum += temp[i];
	}

	if (checksum == msg->checksum)
		return true;
	else
		return false;
}

bool MTF02P::micolink_parse_char(MICOLINK_MSG_t *msg, uint8_t data) {
	switch (msg->status) {
	case 0:
		if (data == MICOLINK_MSG_HEAD) {
			msg->head = data;
			msg->status++;
		}
		break;

	case 1: // device id
		msg->dev_id = data;
		msg->status++;
		break;

	case 2: // system id
		msg->sys_id = data;
		msg->status++;
		break;

	case 3: // message id
		msg->msg_id = data;
		msg->status++;
		break;

	case 4: //
		msg->seq = data;
		msg->status++;
		break;

	case 5: // payload length
		msg->len = data;
		if (msg->len == 0)
			msg->status += 2;
		else if (msg->len > MICOLINK_MAX_PAYLOAD_LEN)
			msg->status = 0;
		else
			msg->status++;
		break;

	case 6: // payload receive
		msg->payload[msg->payload_cnt++] = data;
		if (msg->payload_cnt == msg->len) {
			msg->payload_cnt = 0;
			msg->status++;
		}
		break;

	case 7: // check sum
		msg->checksum = data;
		msg->status = 0;
		if (micolink_check_sum(msg)) {
			return true;
		}

	default:
		msg->status = 0;
		msg->payload_cnt = 0;
		break;
	}

	return false;
}

MTF02P::MTF02P() {
}

void MTF02P::update(uint8_t c) {
	micolink_decode(c);
}

static float wrapDeg(float deg) {
	while (deg > 180.0f)
		deg -= 360.0f;
	while (deg <= -180.0f)
		deg += 360.0f;
	return deg;
}

bool MTF02P::flowUsable() {
	return payload.flow_quality >= FLOW_MIN_QUALITY && payload.distance > 0;
}

void MTF02P::calibrationReset() {
	for (int i = 0; i < FLOW_CAL_DELAYS; i++) {
		calG0G0[i] = calG0G1[i] = calG1G1[i] = 0;
		calFxG0[i] = calFxG1[i] = calFyG0[i] = calFyG1[i] = 0;
	}
	calFxFx = calFyFy = 0;
	rotationSamples = 0;
	forwardSumX = forwardSumY = rightSumX = rightSumY = 0;
	forwardSamples = rightSamples = 0;
}

void MTF02P::calibrationAccumulateRotation(Gyro *g) {
	if (!flowUsable())
		return;
	double fx = payload.flow_vel_x, fy = payload.flow_vel_y;
	calFxFx += fx * fx;
	calFyFy += fy * fy;
	for (int i = 0; i < FLOW_CAL_DELAYS; i++) {
		float g0f, g1f;
		g->gyroDelayed(i * FLOW_CAL_DELAY_STEP_MS, &g0f, &g1f);
		double g0 = g0f, g1 = g1f;
		calG0G0[i] += g0 * g0;
		calG0G1[i] += g0 * g1;
		calG1G1[i] += g1 * g1;
		calFxG0[i] += fx * g0;
		calFxG1[i] += fx * g1;
		calFyG0[i] += fy * g0;
		calFyG1[i] += fy * g1;
	}
	rotationSamples++;
}

void MTF02P::calibrationAccumulateForward() {
	if (!flowUsable())
		return;
	forwardSumX += payload.flow_vel_x;
	forwardSumY += payload.flow_vel_y;
	forwardSamples++;
}

void MTF02P::calibrationAccumulateRight() {
	if (!flowUsable())
		return;
	rightSumX += payload.flow_vel_x;
	rightSumY += payload.flow_vel_y;
	rightSamples++;
}

void MTF02P::calibrationReport() {
	printf("\n--- optical flow calibration (paste into estimations.cpp) ---\n");

	if (rotationSamples < 1000) {
		printf("rotation fit: only %d usable samples\n", rotationSamples);
	} else {
		double n2 = 2.0 * rotationSamples;
		printf("gyro delay sweep, residual rms in flow units (uncompensated %.1f):\n",
				sqrt((calFxFx + calFyFy) / n2));
		int best = -1;
		double bestSse = 0, bestK[4] = { 0 };
		for (int i = 0; i < FLOW_CAL_DELAYS; i++) {
			// 2x2 normal equations per flow axis. det/(G0G0*G1G1) is 1 - corr(g0,g1)^2:
			// it collapses if the rocking only excited one axis or both in lockstep.
			double det = calG0G0[i] * calG1G1[i] - calG0G1[i] * calG0G1[i];
			if (calG0G0[i] < 1e5 || calG1G1[i] < 1e5
					|| det < 0.1 * calG0G0[i] * calG1G1[i]) {
				printf("  %3d ms: not enough independent pitch AND roll excitation\n",
						i * FLOW_CAL_DELAY_STEP_MS);
				continue;
			}
			double kx0 = (calFxG0[i] * calG1G1[i] - calFxG1[i] * calG0G1[i]) / det;
			double kx1 = (calFxG1[i] * calG0G0[i] - calFxG0[i] * calG0G1[i]) / det;
			double ky0 = (calFyG0[i] * calG1G1[i] - calFyG1[i] * calG0G1[i]) / det;
			double ky1 = (calFyG1[i] * calG0G0[i] - calFyG0[i] * calG0G1[i]) / det;
			double sse = (calFxFx - (kx0 * calFxG0[i] + kx1 * calFxG1[i]))
					+ (calFyFy - (ky0 * calFyG0[i] + ky1 * calFyG1[i]));
			if (sse < 0)
				sse = 0;
			printf("  %3d ms: %.1f\n", i * FLOW_CAL_DELAY_STEP_MS, sqrt(sse / n2));
			if (best < 0 || sse < bestSse) {
				best = i;
				bestSse = sse;
				bestK[0] = kx0;
				bestK[1] = kx1;
				bestK[2] = ky0;
				bestK[3] = ky1;
			}
		}
		if (best >= 0) {
			printf("#define FLOW_K_X_G0 %.4ff\n", bestK[0]);
			printf("#define FLOW_K_X_G1 %.4ff\n", bestK[1]);
			printf("#define FLOW_K_Y_G0 %.4ff\n", bestK[2]);
			printf("#define FLOW_K_Y_G1 %.4ff\n", bestK[3]);
			printf("#define FLOW_GYRO_DELAY_MS %d\n", best * FLOW_CAL_DELAY_STEP_MS);
			if (best == FLOW_CAL_DELAYS - 1)
				printf("warning: best delay is at the end of the sweep - true latency may be longer\n");
		}
	}

	if (forwardSamples < 20 || rightSamples < 20) {
		printf("translation: not enough samples (forward %d, right %d)\n",
				forwardSamples, rightSamples);
		return;
	}
	float fx = forwardSumX / forwardSamples, fy = forwardSumY / forwardSamples;
	float rx = rightSumX / rightSamples, ry = rightSumY / rightSamples;

	// body frame is +X right, +Y forward: right x forward must be positive
	int mirror = (rx * fy - ry * fx) < 0;
	if (mirror) {
		fx = -fx;
		rx = -rx;
	}
	// each push is an independent estimate of the twist; a hand push is only straight to
	// a few degrees, so use their mean and report how far apart they are
	float phiForward = 90.0f - atan2f(fy, fx) * RAD_TO_DEG;
	float phiRight = -atan2f(ry, rx) * RAD_TO_DEG;
	float disagree = wrapDeg(phiRight - phiForward);
	float yawOffset = wrapDeg(phiForward + 0.5f * disagree);

	printf("#define FLOW_MIRROR_X %d\n", mirror);
	printf("#define FLOW_YAW_OFFSET_DEG %.2ff\n", yawOffset);
	printf("check: forward/right pushes disagree by %.1f deg (want <5; >15 means a push wasn't straight)\n",
			disagree);
}

void MTF02P::dumpData() {
	printf("%d %d %d %d\n", payload.flow_quality, payload.distance, payload.flow_vel_x, payload.flow_vel_y);
}
