/******************************************************************************
MotorController.cpp
Commands for the MotorController for PID Controller Remote Lab.
David Reid
14/09/20

Contains commands for the DC Motor for the PID Controller remote lab.
******************************************************************************/

#include "MotorController.h"
#include <Arduino.h>

Motor::Motor(int In1pin, int In2pin, int PWMpin, int offset, int STBYpin)
{
  In1 = In1pin;
  In2 = In2pin;
  PWM = PWMpin;
  Standby = STBYpin;
  Offset = offset;
  
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(Standby, OUTPUT);
}

void Motor::drive(int speed)
{
  digitalWrite(Standby, HIGH);
  speed = speed * Offset;
  if (speed>=0) fwd(speed);
  else rev(-speed);
}
void Motor::drive(int speed, int duration)
{
  drive(speed);
  delay(duration);
}

void Motor::fwd(int speed)
{
   digitalWrite(In1, HIGH);
   digitalWrite(In2, LOW);
   analogWrite(PWM, speed);

}

void Motor::rev(int speed)
{
   digitalWrite(In1, LOW);
   digitalWrite(In2, HIGH);
   analogWrite(PWM, speed);
}

void Motor::brake()
{
   digitalWrite(In1, HIGH);
   digitalWrite(In2, HIGH);
   analogWrite(PWM,0);
}

void Motor::free()
{
	digitalWrite(Standby, LOW);	//will this do?
}

void Motor::standby()
{
   digitalWrite(Standby, LOW);
}
