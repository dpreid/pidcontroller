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

#ifndef MotorControllerPmodHB3SAMD21_h
#define MotorControllerPmodHB3SAMD21_h
#include <SAMD21turboPWM.h>

#include <Arduino.h>

//defaults to max speed
#define DEFAULTSPEED 255  



class MotorHB3SAMD21
{
  public:
  MotorHB3SAMD21(int directionPin, int enablePin, int offset, long prescale);      

    //Speed is an integer between -255 and 255.
  void drive(int speed);  
	
  void drive(int speed, int duration);
  
  void setPrescale(long prescale);

  //don't drive motor, but let it spin freely to a stop
  void free();
	
  //Stops motor by setting both input pins high
  void brake(); 
  
  //places the motor in standby mode where commands cannot be sent.
  void standby();

  void setMinTime(int time);

  void setMaxTime(int time);
 
	
  private:
    //PIN variables for the motor controller board
    //2 inputs (AI1 and AI2), PWM input, Offset value, and the Standby pin
    int direction, enable, Offset, prev_speed, servoMinTime, servoMaxTime;
    TurboPWM servo;
    void fwd(int speed);
	void rev(int speed);
    //control the pwm on the enable pin
    void pwm(int speed);
  
};


#endif
