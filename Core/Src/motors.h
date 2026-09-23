#ifndef _MOTORS
#define _MOTORS

#include "main.h"
#include "Constants/constants.h"
#include "Drone/drone_position.h"

#define MIN_PW 1000
#define MAX_PW 2000

class Motors
{
private:
  TIM_HandleTypeDef *htim3, *htim4;
  float lastMotorFL = 0, lastMotorFR = 0, lastMotorRL = 0, lastMotorRR = 0;

public:
  Motors(TIM_HandleTypeDef *htim3, TIM_HandleTypeDef *htim4);
  void writeFL(int intensity);
  void writeFR(int intensity);
  void writeRL(int intensity);
  void writeRR(int intensity);
  void writeAll(int intensity);
  void startup();
  void writeDronePosition(DronePosition* p);
  void testMotors();
};

class MotorsSingleton
{
public:
  static Motors *m;
  static Motors *getInstance(TIM_HandleTypeDef *htim3, TIM_HandleTypeDef *htim4)
  {
    if (m == nullptr)
    {
      m = new Motors(htim3, htim4);
    }

    return m;
  }
};

#endif
