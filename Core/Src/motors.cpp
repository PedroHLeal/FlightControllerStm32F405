#include "motors.h"
#include <stdio.h>

Motors *MotorsSingleton::m = nullptr;

// M1 -> TIM3_CH3
// M2 -> TIM4_CH2
// M3 -> TIM3_CH4
// M4 -> TIM4_CH1

Motors::Motors(TIM_HandleTypeDef *htim3, TIM_HandleTypeDef *htim4)
{
	this->htim3 = htim3;
	this->htim4 = htim4;
	HAL_TIM_PWM_Start(htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(htim3, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(htim4, TIM_CHANNEL_2);
}

void Motors::writeFL(int intensity)
{
    if (intensity < 0 || intensity > 180)
    {
    	intensity = 0;
    }
	int n_intensity = map(intensity, 0, 180, 1000, 2000);
    __HAL_TIM_SET_COMPARE(htim3, TIM_CHANNEL_3, n_intensity);
}

void Motors::writeFR(int intensity)
{
	if (intensity < 0 || intensity > 180)
	{
		intensity = 0;
	}
	int n_intensity = map(intensity, 0, 180, 1000, 2000);
    __HAL_TIM_SET_COMPARE(htim4, TIM_CHANNEL_2, n_intensity);
}

void Motors::writeRL(int intensity)
{
	if (intensity < 0 || intensity > 180)
	{
		intensity = 0;
	}
	int n_intensity = map(intensity, 0, 180, 1000, 2000);
    __HAL_TIM_SET_COMPARE(htim3, TIM_CHANNEL_4, n_intensity);
}

void Motors::writeRR(int intensity)
{
	if (intensity < 0 || intensity > 180)
	{
		intensity = 0;
	}
	int n_intensity = map(intensity, 0, 180, 1000, 2000);
    __HAL_TIM_SET_COMPARE(htim4, TIM_CHANNEL_1, n_intensity);
}

void Motors::writeAll(int intensity)
{
	int n_intensity = map(intensity, 0, 180, 1000, 2000);
	__HAL_TIM_SET_COMPARE(htim3, TIM_CHANNEL_3, n_intensity);
	__HAL_TIM_SET_COMPARE(htim3, TIM_CHANNEL_4, n_intensity);
	__HAL_TIM_SET_COMPARE(htim4, TIM_CHANNEL_1, n_intensity);
	__HAL_TIM_SET_COMPARE(htim4, TIM_CHANNEL_2, n_intensity);
}

void Motors::startup() {
	__HAL_TIM_SET_COMPARE(htim3, TIM_CHANNEL_3, 2000);
	__HAL_TIM_SET_COMPARE(htim3, TIM_CHANNEL_4, 2000);
	__HAL_TIM_SET_COMPARE(htim4, TIM_CHANNEL_1, 2000);
	__HAL_TIM_SET_COMPARE(htim4, TIM_CHANNEL_2, 2000);

	HAL_Delay(5000);

	__HAL_TIM_SET_COMPARE(htim3, TIM_CHANNEL_3, 1000);
	__HAL_TIM_SET_COMPARE(htim3, TIM_CHANNEL_4, 1000);
	__HAL_TIM_SET_COMPARE(htim4, TIM_CHANNEL_1, 1000);
	__HAL_TIM_SET_COMPARE(htim4, TIM_CHANNEL_2, 1000);

	HAL_Delay(10000);
}

void Motors::writeDronePosition(DronePosition *p)
{
//	printf("%d %d %.3f %.3f %.3f\n", 30, -30, p->pitch, p->roll, p->yaw);
    float frontIntensity = -p->pitch;
    float rearIntensity = p->pitch;
    float rightIntensity = p->roll;
    float leftIntensity = -p->roll;

    float motorFL = p->throttle + leftIntensity + frontIntensity + p->yaw;
    float motorFR = p->throttle + rightIntensity + frontIntensity - p->yaw;
    float motorRL = p->throttle + leftIntensity + rearIntensity - p->yaw;
    float motorRR = p->throttle + rightIntensity + rearIntensity + p->yaw;
//    printf("%.2f %.2f %.2f %.2f\n", constrain(motorFL, 0, 180), constrain(motorFR, 0, 180), constrain(motorRL, 0, 180),constrain(motorRR, 0, 180));

    writeFL(constrain(motorFL, 0, 180));
    writeFR(constrain(motorFR, 0, 180));
    writeRL(constrain(motorRL, 0, 180));
    writeRR(constrain(motorRR, 0, 180));
}

void Motors::testMotors()
{
    writeFL(30);
    HAL_Delay(1000);
    writeFR(30);
    HAL_Delay(1000);
    writeRL(30);
    HAL_Delay(1000);
    writeRR(30);
    HAL_Delay(99999999);
}
