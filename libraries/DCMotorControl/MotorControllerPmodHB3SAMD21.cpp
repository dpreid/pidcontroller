/******************************************************************************
MotorController.cpp
Commands for the MotorController for PID Controller Remote Lab.
David Reid
14/09/20

Modified Tim Drysdale
18 Feb 2021
Use SAMD21 PWM

Contains commands for the DC Motor for the PID Controller remote lab.
******************************************************************************/

#include "MotorControllerPmodHB3SAMD21.h"
#include <Arduino.h>

MotorHB3SAMD21::MotorHB3SAMD21(int directionPin, int enablePin, int offset, long prescale)
{
  direction = directionPin;
  enable = enablePin;
  Offset = offset;
  
  prev_speed = 0;
  
  pinMode(direction, OUTPUT);
  pinMode(enable, OUTPUT);

 
  servo.setClockDivider(1, false);  // Input clock is divided by 1 and 48MHz is sent to Generic Clock, Turbo is off
  servo.timer(0, 1, prescale, true); //timer0 for pin6 https://github.com/ocrdu/Arduino_SAMD21_turbo_PWM, was 960000 for 48MHz/960000=50Hz

  servoMinTime = 0;
  servoMaxTime = 20000;
    
}

void MotorHB3SAMD21::setMinTime(int time) {
  servoMinTime = time;
}

void MotorHB3SAMD21::setMaxTime(int time) {
  servoMaxTime = time;
}

void MotorHB3SAMD21::pwm(int speed) {

  //convert speed to microseconds 0 -> servoMinTime, 255 -> servoMaxTime
  
  int microSeconds = servoMinTime + (abs(speed) * (servoMaxTime-servoMinTime) / 255);
  //float time_range = servoMaxTime-servoMinTime;
  //float time_added = relative_speed * time_range;
  //int microSeconds = int(time_added) + servoMinTime;
  //int microSeconds = speed * 30;
  servo.analogWrite(enable, microSeconds / 20); // 1sec/20 = 50Hz

}

void MotorHB3SAMD21::drive(int speed)
{
  if(speed / prev_speed < 0) free();	//H bridge cannot have enable pin high when direction changes 

  speed = speed * Offset;
  if (speed>=0) fwd(speed);
  else rev(-speed);
  
  prev_speed = speed;
}

void MotorHB3SAMD21::drive(int speed, int duration)
{
  drive(speed);
  delay(duration);
}

void MotorHB3SAMD21::fwd(int speed)
{
   digitalWrite(direction, LOW);
   pwm(speed);
}

void MotorHB3SAMD21::rev(int speed)
{
   digitalWrite(direction, HIGH);
   pwm(speed);
}

void MotorHB3SAMD21::brake()
{
  free();
}

void MotorHB3SAMD21::free()
{
  // must set enable to zero or else changing direction will
  // damage the h-bridge
  servo.analogWrite(enable, 0);
}

void MotorHB3SAMD21::standby()
{
  free();
}
