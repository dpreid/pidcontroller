/******************************************************************************
PIDMotorController.h
Hardware: DC Motor and H bridge motor controller
David Reid
14/09/20

DC Motor controller functions for PID Controller Remote Lab.
******************************************************************************/

#ifndef MotorController_h
#define MotorController_h

#include <Arduino.h>

//defaults to max speed
#define DEFAULTSPEED 255  



class Motor
{
  public:
    Motor(int In1pin, int In2pin, int PWMpin, int offset, int STBYpin);      

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
	int In1, In2, PWM, Offset,Standby;
    void fwd(int speed);
	void rev(int speed);

};


#endif
