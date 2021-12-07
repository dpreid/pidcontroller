/******************************************************************************
MotorController.cpp
Commands for the MotorController for PID Controller Remote Lab.
David Reid
14/09/20

Modified Tim Drysdale
18 Feb 2021
Use SAMD21 PWM

Modified David Reid
24 Sep 2021
Modified for BTN8982 and removed SAMD PWM

Contains commands for the DC Motor for the PID Controller remote lab.
******************************************************************************/

#include "MotorControllerBTN8982.h"
#include <Arduino.h>

MotorBTN8982::MotorBTN8982(int enablePin, int leftPWMPin, int rightPWMPin, int dir, int maxVolts)
{
  enable = enablePin;
  leftPWM = leftPWMPin;
  rightPWM = rightPWMPin;

  direction = dir;
  maxVoltage = maxVolts;
  
  prevVoltage = 0;
  
  pinMode(enable, OUTPUT);
  pinMode(leftPWM, OUTPUT);
  pinMode(rightPWM, OUTPUT);
  digitalWrite(enable,LOW);
  digitalWrite(leftPWM,LOW);
  digitalWrite(rightPWM,LOW);

    
}

//Duty cycle between 0 and 255 for analogue write.
unsigned int MotorBTN8982::voltageToDuty(float voltage) {
  
  // convert
  // -1.0 <= speed <= +1.0
  // to duty cycle unsigned int 0 - 1000 (0 - 100% in 0.1% steps?)
  voltage = abs(voltage);
  
  if (voltage > maxVoltage) {
	voltage = maxVoltage;
  }
  
  return (unsigned int) (255.0f * voltage / maxVoltage);
}

void MotorBTN8982::drive(float voltage)
{
  if(voltage / prevVoltage < 0) free();	//H bridge cannot have enable pin high when direction changes
  // This library gives 9.7uS offtime between directions running on IOT nano 33.
  
  voltage = voltage * direction;
  if (voltage>=0) fwd(voltage);
  else rev(voltage);
  
  prevVoltage = voltage;
}

void MotorBTN8982::fwd(float voltage)
{
  analogWrite(leftPWM, 0);
  analogWrite(rightPWM, voltageToDuty(voltage));
  digitalWrite(enable,HIGH);
}

void MotorBTN8982::rev(float voltage)
{
  analogWrite(rightPWM, 0);
  analogWrite(leftPWM, voltageToDuty(voltage));
  digitalWrite(enable,HIGH);
}

void MotorBTN8982::brake()
{
  free();
}

void MotorBTN8982::free()
{
  // must set enable to zero or else changing direction will
  // damage the h-bridge
  digitalWrite(enable,LOW);
  analogWrite(leftPWM, 0);
  analogWrite(rightPWM, 0);
  
}

void MotorBTN8982::standby()
{
  free();
}
