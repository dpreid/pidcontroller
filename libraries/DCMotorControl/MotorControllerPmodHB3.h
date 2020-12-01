/******************************************************************************
PIDMotorController.h
Hardware: DC Motor and H bridge motor controller
David Reid
14/09/20

DC Motor controller functions for PID Controller Remote Lab.
******************************************************************************/

#ifndef MotorControllerPmodHB3_h
#define MotorControllerPmodHB3_h

#include <Arduino.h>

//defaults to max speed
#define DEFAULTSPEED 255  



class MotorHB3
{
  public:
    MotorHB3(int directionPin, int enablePin, int offset);      

    //Speed is an integer between -255 and 255.
    void drive(int speed);  
	
    void drive(int speed, int duration);  
	
    //don't drive motor, but let it spin freely to a stop
    void free();
	
	//Stops motor by setting both input pins high
    void brake(); 
	
    //places the motor in standby mode where commands cannot be sent.
	void standby();	
	
  private:
    //PIN variables for the motor controller board
    //2 inputs (AI1 and AI2), PWM input, Offset value, and the Standby pin
	int direction, enable, Offset, prev_speed;
    void fwd(int speed);
	void rev(int speed);

};


#endif
