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

#include "MotorControllerBTS7960.h"
#include <Arduino.h>

MotorBTS7960::MotorBTS7960(int enablePin, int leftPWMPin, int rightPWMPin, int timerNumber, int offset, long prescale)
{
  enable = enablePin;
  leftPWM = leftPWMPin;
  rightPWM = rightPWMPin;

  timer = timerNumber;
  
  Offset = offset;
  
  prevSpeed = 0;
  
  pinMode(enable, OUTPUT);
  pinMode(leftPWM, OUTPUT);
  pinMode(rightPWM, OUTPUT);
  digitalWrite(enable,LOW);
  digitalWrite(leftPWM,LOW);
  digitalWrite(rightPWM,LOW);
 
  servo.setClockDivider(1, false);  // Input clock is divided by 1 and 48MHz is sent to Generic Clock, Turbo is off
  servo.timer(timer, 1, prescale, true); //timer0 for pin6, timer1 for 4&7 https://github.com/ocrdu/Arduino_SAMD21_turbo_PWM, was 960000 for 48MHz/960000=50Hz

  minSpeed = 0;
  maxSpeed = MAX_ABS_SPEED;
    
}

void MotorBTS7960::setPrescale(long prescale) {
  servo.timer(timer, 1, prescale, true);
}


unsigned int MotorBTS7960::speedToDuty(float speed) {
  
  // convert
  // -1.0 <= speed <= +1.0
  // to duty cycle unsigned int 0 - 1000 (0 - 100% in 0.1% steps?)
  speed = abs(speed);
  
  if (speed > 1) {
	speed = 1.0;
  }

  speed = minSpeed + (speed * (maxSpeed - minSpeed));
  
  return (unsigned int) (1000.0f * speed);
}

void MotorBTS7960::drive(float speed)
{
  if(speed / prevSpeed < 0) free();	//H bridge cannot have enable pin high when direction changes
  // This library gives 9.7uS offtime between directions running on IOT nano 33.
  
  speed = speed * Offset;
  if (speed>=0) fwd(speed);
  else rev(-speed);
  
  prevSpeed = speed;
}

void MotorBTS7960::fwd(float speed)
{
  servo.analogWrite(leftPWM, 0);
  servo.analogWrite(rightPWM, speedToDuty(speed));
  digitalWrite(enable,HIGH);
}

void MotorBTS7960::rev(float speed)
{
  servo.analogWrite(rightPWM, 0);
  servo.analogWrite(leftPWM, speedToDuty(speed));
  digitalWrite(enable,HIGH);
}

void MotorBTS7960::brake()
{
  free();
}

void MotorBTS7960::free()
{
  // must set enable to zero or else changing direction will
  // damage the h-bridge
  digitalWrite(enable,LOW);
  servo.analogWrite(leftPWM, 0);
  servo.analogWrite(rightPWM, 0);
  
}

void MotorBTS7960::standby()
{
  free();
}
