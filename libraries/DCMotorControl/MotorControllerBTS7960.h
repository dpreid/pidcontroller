/******************************************************************************
PIDMotorController.h
Hardware: DC Motor and H bridge motor controller
David Reid
14/09/20

Modified Tim Drysdale
18 Feb 2021
Use SAMD21 PWM

DC Motor controller functions for PID Controller Remote Lab.
******************************************************************************/

#ifndef DCMOTORCONTROL_DCMOTORCONTROLLERBTS7960_h
#define DCMOTORCONTROL_DCMOTORCONTROLLERBTS7960_h

#include <SAMD21turboPWM.h>
#include <Arduino.h>

#define MAX_ABS_SPEED 1.0
  
class MotorBTS7960
{
  public:
  MotorBTS7960(int enablePin, int leftPWMPin, int rightPWMPin, int timerNumber, int offset, long prescale);

    //Speed is an integer between -255 and 255.
  void drive(float speed);  
  
  void setPrescale(long prescale);

  //don't drive motor, but let it spin freely to a stop
  void free();
	
  //Stops motor by setting both input pins high
  void brake(); 
  
  //places the motor in standby mode where commands cannot be sent.
  void standby();

  float minSpeed, maxSpeed;
  
  private:
  //PIN variables for the motor controller board
  int enable, leftPWM, rightPWM, timer;

  //direction adjustment
  int Offset;
 
  float prevSpeed; //last speed
  
  TurboPWM servo;
  unsigned int speedToDuty(float speed);
  void fwd(float speed);
  void rev(float speed);

  
};


#endif
