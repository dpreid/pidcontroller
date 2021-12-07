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

#ifndef DCMOTORCONTROL_DCMOTORCONTROLLERBTN8982_h
#define DCMOTORCONTROL_DCMOTORCONTROLLERBTN8982_h

#include <Arduino.h>
  
class MotorBTN8982
{
  public:
  MotorBTN8982(int enablePin, int leftPWMPin, int rightPWMPin, int direction, int maxVoltage);

    //Voltage is a float between -12 and +12.
  void drive(float voltage);  

  //don't drive motor, but let it spin freely to a stop
  void free();
	
  //Stops motor by setting both input pins high
  void brake(); 
  
  //places the motor in standby mode where commands cannot be sent.
  void standby();
  
  private:
  //PIN variables for the motor controller board
  int enable, leftPWM, rightPWM;

  //direction adjustment
  int direction;
  
  int maxVoltage;
 
  float prevVoltage; //last speed
  
  unsigned int voltageToDuty(float voltage);
  void fwd(float voltage);
  void rev(float voltage);

  
};


#endif
