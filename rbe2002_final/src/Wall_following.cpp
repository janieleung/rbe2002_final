#include <Romi32U4.h>
#include "Encoders.h"
#include "Wall_following.h"
#include "IR_sensor.h"
#include "Sonar_sensor.h"
#include "Position_estimation.h"

IRsensor SharpIR;
SonarSensor HCSR04;

void WallFollowingController::Init(void)
{
    SharpIR.Init();
    HCSR04.Init();
}

float WallFollowingController::Process(float target_distance)
{
  //assignment 2: write a PD controller that outputs speed as a function of distance error
  float e_distance = target_distance - SharpIR.ReadData();
  float d_e = e_distance - prev_e_distance;

  sumOFE_distance += e_distance;

  float speed = constrain(Kp * e_distance + Kd * d_e, -40, 40);
  prev_e_distance = e_distance;

  return speed;
}