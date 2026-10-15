/*
 * battery.h
 *
 * Battery voltage/current on the SpeedyBee F405 AIO dividers (ADC1 IN10 = PC0 voltage,
 * IN12 = PC2 current), read at register level so no HAL ADC module is needed.
 */

#ifndef SRC_SENSORS_BATTERY_H_
#define SRC_SENSORS_BATTERY_H_

#define VBAT_DIVIDER 11.0f      // board divider ratio (ArduPilot BATT_VOLT_MULT / Betaflight vbat_scale 110)
#define VBAT_CAL 1.007f         // measured / reported: multimeter 11.83 V vs raw 11.75 V (2026-10-07)
#define IBAT_AMP_PER_VOLT 25.9f   // SpeedyBee F405 Mini: Betaflight ibata_scale 386 = 38.6 mV/A; uncalibrated, trim via charger mAh
#define VBAT_FILTER_TAU 0.5f    // seconds

// Motor commands are scaled by VBAT_REF_V / vbat so thrust per command stays constant as the
// pack sags; the tune is then "as at VBAT_REF_V". 0 disables.
#define VBAT_SAG_COMP 1
#define VBAT_REF_V 11.4f        // 3.8 V/cell on 3S
#define VBAT_SAG_COMP_MIN 0.85f
#define VBAT_SAG_COMP_MAX 1.35f

void batteryInit();
void batteryUpdate(float dt);   // once per control tick
float batteryVoltage();         // filtered, V
float batteryCurrent();         // filtered, A (uncalibrated offset)
bool batteryValid();
float batterySagCompensation(); // multiplier for motor commands, 1.0 when disabled/invalid

#endif /* SRC_SENSORS_BATTERY_H_ */
