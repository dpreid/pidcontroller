/******************************************************************************
MotorController.cpp
Commands for the MotorController for PID Controller Remote Lab.
David Reid
14/09/20

Contains commands for the DC Motor for the PID Controller remote lab.
******************************************************************************/

#include "MotorControllerPmodHB3.h"
#include <Arduino.h>

MotorHB3::MotorHB3(int directionPin, int enablePin, int offset)
{
  direction = directionPin;
  enable = enablePin;
  Offset = offset;
  
  prev_speed = 0;
  
  pinMode(direction, OUTPUT);
  pinMode(enable, OUTPUT);
}

void MotorHB3::drive(int speed)
{
if(speed / prev_speed < 0) analogWrite(enable, 0);	//H bridge cannot have enable pin high when direction changes 

  speed = speed * Offset;
  if (speed>=0) fwd(speed);
  else rev(-speed);
  
  prev_speed = speed;
}
void MotorHB3::drive(int speed, int duration)
{
  drive(speed);
  delay(duration);
}

void MotorHB3::fwd(int speed)
{
   digitalWrite(direction, LOW);
   analogWrite(enable, speed);

}

void MotorHB3::rev(int speed)
{
   digitalWrite(direction, HIGH);
   analogWrite(enable, speed);
}

void MotorHB3::brake()
{  
   analogWrite(enable,0);
}

void MotorHB3::free()
{
	
}

void MotorHB3::standby()
{
   
}
