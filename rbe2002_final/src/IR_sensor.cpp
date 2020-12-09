#include <Romi32U4.h>
#include "IR_sensor.h"

void IRsensor::Init(void)
{
    pinMode(pin_IR, INPUT);
}

float IRsensor::PrintData(void)
{
    Serial.println(ReadData());
}

float IRsensor::ReadData(void)
{
  //assignment 1.1
  //read out and calibrate your IR sensor, to convert readouts to distance in [cm]
  int x = analogRead(A0);
  float y = 6.48813*pow(10,-9)*pow(x,4) - 8.98867*pow(10,-6)*pow(x,3) + 0.00468652*pow(x,2) - 1.14575*x +129.441;
  return y;
}