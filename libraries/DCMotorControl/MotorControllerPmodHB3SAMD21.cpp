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
  
  prevSpeed = 0;
  
  pinMode(direction, OUTPUT);
  pinMode(enable, OUTPUT);

 
  servo.setClockDivider(1, false);  // Input clock is divided by 1 and 48MHz is sent to Generic Clock, Turbo is off
  servo.timer(0, 1, prescale, true); //timer0 for pin6 https://github.com/ocrdu/Arduino_SAMD21_turbo_PWM, was 960000 for 48MHz/960000=50Hz

  minSpeed = 0;
  maxSpeed = MAX_ABS_SPEED;
    
}

void MotorHB3SAMD21::setPrescale(long prescale) {
  servo.timer(0, 1, prescale, true);
}


unsigned int MotorHB3SAMD21::speedToDuty(float speed) {
  
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

void MotorHB3SAMD21::pwm(float speed) {
    
  servo.analogWrite(enable, speedToDuty(speed)); 

}

void MotorHB3SAMD21::drive(float speed)
{
  if(speed / prevSpeed < 0) free();	//H bridge cannot have enable pin high when direction changes 

  speed = speed * Offset;
  if (speed>=0) fwd(speed);
  else rev(-speed);
  
  prevSpeed = speed;
}

void MotorHB3SAMD21::drive(float speed, int duration)
{
  drive(speed);
  delay(duration);
}

void MotorHB3SAMD21::fwd(float speed)
{
   digitalWrite(direction, LOW);
   pwm(speed);
}

void MotorHB3SAMD21::rev(float speed)
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
