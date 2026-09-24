/*
 * mtp02p.h
 *
 *  Created on: Apr 29, 2025
 *      Author: pedroleal
 */

#ifndef SRC_SENSORS_MTP02P_H_
#define SRC_SENSORS_MTP02P_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "main.h"
#include "Gyro.h"

#define MICOLINK_MSG_HEAD 0xEF
#define MICOLINK_MAX_PAYLOAD_LEN 64
#define MICOLINK_MAX_LEN MICOLINK_MAX_PAYLOAD_LEN + 7

#define FLOW_MIN_QUALITY 70

// calibration sweeps gyro delays 0 .. (FLOW_CAL_DELAYS-1)*STEP ms
#define FLOW_CAL_DELAYS 21
#define FLOW_CAL_DELAY_STEP_MS 10

/*
    Message ID
*/
enum
{
    MICOLINK_MSG_ID_RANGE_SENSOR = 0x51, // Range Sensor
};

/*
    Message Structure Definition
*/
typedef struct
{
    uint8_t head;
    uint8_t dev_id;
    uint8_t sys_id;
    uint8_t msg_id;
    uint8_t seq;
    uint8_t len;
    uint8_t payload[MICOLINK_MAX_PAYLOAD_LEN];
    uint8_t checksum;

    uint8_t status;
    uint8_t payload_cnt;
} MICOLINK_MSG_t;

/*
    Payload Definition
*/
#pragma pack(1)
// Range Sensor
typedef struct
{
    uint32_t time_ms;     // System time in ms
    uint32_t distance;    // distance(mm), 0 Indicates unavailable
    uint8_t strength;     // signal strength
    uint8_t precision;    // distance precision
    uint8_t dis_status;   // distance status
    uint8_t reserved1;    // reserved
    int16_t flow_vel_x;   // optical flow velocity in x
    int16_t flow_vel_y;   // optical flow velocity in y
    uint8_t flow_quality; // optical flow quality
    uint8_t flow_status;  // optical flow status
    uint16_t reserved2;   // reserved
} MICOLINK_PAYLOAD_RANGE_SENSOR_t;
#pragma pack()

class MTF02P
{
private:
    UART_HandleTypeDef *huart6;

    // least-squares sums of raw sensor flow vs gyro, one set per candidate gyro delay
    double calG0G0[FLOW_CAL_DELAYS], calG0G1[FLOW_CAL_DELAYS], calG1G1[FLOW_CAL_DELAYS];
    double calFxG0[FLOW_CAL_DELAYS], calFxG1[FLOW_CAL_DELAYS];
    double calFyG0[FLOW_CAL_DELAYS], calFyG1[FLOW_CAL_DELAYS];
    double calFxFx = 0, calFyFy = 0;
    int rotationSamples = 0;
    float forwardSumX = 0, forwardSumY = 0, rightSumX = 0, rightSumY = 0;
    int forwardSamples = 0, rightSamples = 0;

    bool micolink_decode(uint8_t data);
    bool micolink_check_sum(MICOLINK_MSG_t *msg);
    bool micolink_parse_char(MICOLINK_MSG_t *msg, uint8_t data);

public:
    MICOLINK_MSG_t msg;

    MICOLINK_PAYLOAD_RANGE_SENSOR_t payload;
    MTF02P();
    void update(uint8_t c);
    bool flowUsable();
    void calibrationReset();
    void calibrationAccumulateRotation(Gyro *g);
    void calibrationAccumulateForward();
    void calibrationAccumulateRight();
    void calibrationReport();
    void dumpData();
};

#endif /* SRC_SENSORS_MTP02P_H_ */
