
/*
 * Remote Lab: Spinning Disk Controller
 * David Reid
 * 02/12/20
 * 
 * Based upon Tim Drysdale's penduino code.
 * https://github.com/timdrysdale/penduino
 * 
 * Modifed by Tim Drysdale Feb 2021
 *
 */


//=============================================================
// SET BOTH THESE TO FALSE BEFORE ROLLING OUT IN PRODUCTION

// report additional information (may affect performance)
bool debug = false;

// enable on-the-fly setting of additional tuning parameters
bool development = true;

//=============================================================


/******** DC MOTOR ******/
#include <MotorControllerPmodHB3SAMD21.h>

// Pins connecting to drive board
#define AIN1 5
#define PWMA 6

const int offset = 1; // If motor spins in the opposite direction then you can change this to -1.

MotorHB3SAMD21 motor = MotorHB3SAMD21(AIN1, PWMA, offset, 240000); //240000 for 200Hz PWM, 480000 for 100Hz PWM, 960000 for 50Hz


float DCMotorMaxRPS = 40; // this is an estimated value, for information purposes 

// Friction compensation
bool enableFrictionComp = true;
float frictionCompStaticCW = (1.8/12.0)*255; //was 1.8
float frictionCompStaticCCW = (1.7/12.0)*255;  //was 1.7
float frictionCompWindow = 5.0;
float frictionCompDynamicCW = (1.0/12.0)*255; //was 1.0
float frictionCompDynamicCCW = (0.9/12.0)*255; //was 0.9

// Timer - to switch off motor at end of a run
const float secondsToMillis = 1000;
float startedModeAtTime = 0;        //ms
float shutdownTimerMillis = 180 * secondsToMillis;     
float longestShutdownTimerSeconds = 180;

/****** ENCODER *******/
#include <Encoder.h>

#define encoderPinA 3     //these pins all have interrupts on them.
#define encoderPinB 2
#define indexPin 11

Encoder encoder(encoderPinA, encoderPinB);
const float encoderPPR = 500;
const float LPFCoefficient = 0.1;
const float timeToSeconds = 1e-6;

RotaryPlant disk = RotaryPlant(encoderPPR, LPFCoefficient, timeToSeconds);


const float positionLimit = 2 * encoderPPR; // using both edges on A & B lines for position
const float encoderMin = positionLimit * -1; // negative limit
const float encoderMax = positionLimit -1;   // assign zero to position half of the rotation, so subtract one

volatile float speed_angular_velocity = 0; //for speed mode velocity reporting
unsigned long speed_current_time_encoder = 0;
unsigned long speed_previous_time_encoder = 0;

const float positionEpsilon = 1.0f / (float)encoderPPR;
const float speedEpsilon = positionEpsion; //used to check if command is close to zero, value not critical.

/****** INTERRUPTS *******/
// flags for ISR to trigger heavy-weight tasks to run in main loop
volatile bool doReport = false;
volatile bool doPID = false;


/******** PID CONTROL ********/
include <pid.h>

//PID parameters
float Kp = 1.0;              
float Ki = 0.0;
float Kd = 0.0;
float Ts = 0.02;
float N = 20;
float uMin = -1;
float uMax = +1;

PID controller = PID(Kp,Ki,Kd,Ts,N,uMin,uMax);

const float minKp = 0;
const float maxKp = 999;
const float minKi = 0;
const float maxKi = 999;
const float minKd = 0;
const float maxKd = 999;
const float minTs = 1;
const float maxTs = 1000;

float PIDInterval = Ts * 1000;  //ms, for timer interrupt 

// Timer
#define CPU_HZ 48000000
#define TIMER_PRESCALER_DIV 1024
float timer_interrupt_freq = 1000.0/PIDInterval;  


/******* SERIAL COMMANDS *******/
#include "ArduinoJson-v6.9.1.h"

//JSON serialization
#define COMMAND_SIZE 128  // was 64
StaticJsonDocument<COMMAND_SIZE> doc;
char command[COMMAND_SIZE];

// Data reporting format
#define SHOW_NONE 0
#define SHOW_PLAIN 1
#define SHOW_SHORT 2
#define SHOW_SHORT_SPEED 3
#define SHOW_SHORT_POS 4
#define SHOW_LONG 5
#define SHOW_FC 6

volatile int show_mode = SHOW_LONG;

int report_integer = 1;          //an integer multiple of the PIDInterval for reporting data set to 5 for speed modes, 1 for position mode.
int report_count = 0;

/******** OTHER GLOBAL VARIABLES ********/
int loop_count = 0; //check if needed?
unsigned long dta; // moving average, needs right shifting by 3 bits to get correct value


// SM Awaiting Stop
int awaitingStopP0 = 0;
int awaitingStopP1 = 0;  
int awaitingStopCount = 100;   
int awaitingStopNeeded = 100;  //number of loops with the same encoder position to assume that motor is stopped, at rate of loop() (much faster than PID rate).

// SMStateChange<MODE>Coefficients
float newKp = 1.0;
float newKi = 0.0;
float newKd = 0.0;
float newTs = 10;
bool isNewKp = false;
bool isNewKi = false;
bool isNewKd = false;
bool isNewTs = false;

// SMStateChangeDCMotor
float DCMotorMaxSpeed = 1.0;
float DCMotorChangeSpeed = 0;
float DCMotorSetSpeed = 0;

// SMStateChangePIDPosition
float changePIDPosition = 0;

// SMStateChangePIDSpeed
float changePIDSpeed = 0;



//=============================================================
// Function Prototypes
//=============================================================


bool stopPIDSpeed(void);
bool stopRawSpeed(void);
bool stopPIDPosition(void);
void newShutdownTimer(float time);
void doEncoderAPosition();
void doEncoderASpeed();
void doEncoderBPosition();
void encoderWrap(void);
void inRange(float val, float min, float max);
void changePIDCoefficients(void);
void resetPIDSignal(void);
void calculateSpeedPID(void);
void calculatePositionPID(void);
float lowpass_filter(float input, float previous_output);
float deadband(float input, float band);
float friction_compensation_static(float drive_signal, float encoderAngVel, float error);
float friction_compensation_dynamic(float drive_signal, float encoderAngVel, float error);
void updateDrivePosition(void);
void updateDriveSpeed(void);
float limitDrivePosition(float drive);
float limitDriveSpeed(float drive);
float limit(float val, float min, float max);
float addFC(float drive);
void calculatePID(void);
void maybeCalculatePID(void);
void maybeReport(void);



/**
 * Defines the valid states for the state machine
 * 
 */
typedef enum
{
  STATE_STOPPING_BEFORE,
  STATE_STOPPING_DURING,  //checking if the motor has stopped
  STATE_STOPPING_AFTER,          //no drive to motor
  STATE_MOTOR_BEFORE,
  STATE_SPEED_DURING, //pid controller mode - 1st order, speed functions
  STATE_SPEED_CHANGE_COMMAND,
  STATE_SPEED_CHANGE_PARAM,
  STATE_SPEED_AFTER,
  STATE_SPEED_BEFORE,
  STATE_SPEED_DURING, //pid controller mode - 1st order, speed functions
  STATE_SPEED_CHANGE_COMMAND,
  STATE_SPEED_CHANGE_PARAM,
  STATE_SPEED_AFTER,
  STATE_POSITION_BEFORE,
  STATE_POSITION_DURING, //pid controller mode - 1st order, speed functions
  STATE_POSITION_CHANGE_COMMAND,
  STATE_POSITION_CHANGE_PARAM,
  STATE_POSITION_AFTER,
} StateType;

//state Machine function prototypes
//these are the functions that run whilst in each respective state.
void stateStoppingBefore(void);
void stateStoppingDuring(void);
void stateStoppingAfter(void);
void stateMotorBefore(void);
void stateSpeedDuring(void);
void stateSpeedChangeCommand(void);
void stateSpeedChangeParam(void);
void stateSpeedAfter(void);
void stateSpeedBefore(void);
void stateSpeedDuring(void);
void stateSpeedChangeCommand(void);
void stateSpeedChangeParam(void);
void stateSpeedAfter(void);
void statePositionBefore(void);
void statePositionDuring(void);
void statePositionChangeCommand(void);
void statePositionChangeParam(void);
void statePositionAfter(void);                   

/**
 * Type definition used to define the state
 */
 
typedef struct
{
  StateType State; /**< Defines the command */
  void (*func)(void); /**< Defines the function to run */
} StateMachineType;

/**
 * A table that defines the valid states of the state machine and
 * the function that should be executed for each state
 */
StateMachineType StateMachine[] =
  { 
  {STATE_STOPPING_BEFORE,                   stateStoppingBefore};                                
  {STATE_STOPPING_DURING, 				   stateStoppingDuring};                  
  {STATE_STOPPING_AFTER,  				   stateStoppingAfter};                   
  {STATE_MOTOR_BEFORE,					   stateMotorBefore};                     
  {STATE_SPEED_DURING, 					   stateSpeedDuring};                     
  {STATE_SPEED_CHANGE_COMMAND,			   stateSpeedChangeCommand};              
  {STATE_SPEED_CHANGE_PARAM,				   stateSpeedChangeParam};                
  {STATE_SPEED_AFTER,					   stateSpeedAfter};                      
  {STATE_SPEED_BEFORE,					   stateSpeedBefore};                     
  {STATE_SPEED_DURING, 					   stateSpeedDuring};                     
  {STATE_SPEED_CHANGE_COMMAND,			   stateSpeedChangeCommand};              
  {STATE_SPEED_CHANGE_PARAM,				   stateSpeedChangeParam};                
  {STATE_SPEED_AFTER,					   stateSpeedAfter};                      
  {STATE_POSITION_BEFORE,				   statePositionBefore};                  
  {STATE_POSITION_DURING, 				   statePositionDuring};                  
  {STATE_POSITION_CHANGE_COMMAND,		   statePositionChangeCommand};           
  {STATE_POSITION_CHANGE_PARAM,			   statePositionChangeParam};             
  {STATE_POSITION_AFTER,					   statePositionAfter};                   
};
 
int numStates = 17;

/**
 * Stores the current state of the state machine
 */
 
StateType SMState = STATE_STOPPED;    //START IN THE STOPPED STATE

//DEFINE STATE MACHINE FUNCTIONS================================================================

void SMStateStopped(void){        

  SMState = STATE_STOPPED; //default next state

  setPointPosition = encoderPos;
  setPointSpeed = 0;
  encoderAngVel = 0;

}

void SMStateBeforeAwaitingStop(void){

  SMState = STATE_AWAITING_STOP; //default next state
 
}

void SMStateAwaitingStop(void){

  // we coast to a stop for convenience because active braking with PIDPosition mode might end up oscillating

  SMState = STATE_AWAITING_STOP; //default next state
 
  motor.brake();

  awaitingStopP1 = awaitingStopP0;
  awaitingStopP0 = disk.getPosition();

  if (cmpf(awaitingStopP0, awaitingStopP1, positionEpsilon)) {
	awaitingStopCount -= 1;
  }
  else{
	awaitingStopCount = awaitingStopNeeded;
  }

  maybeReport();
  
  if (awaitingStopCount <= 0){
	awaitingStopCount = awaitingStopNeeded; // avoid edge case on next stop event where enter stop mode at moment direction is reversing and encoder momentarily same value
	SMState = STATE_STOPPED;   
  } 
}


void SMStateBeforeDCMotor(void){

  SMState = STATE_DC_MOTOR_MODE; //default next state
  
  report_integer = 5;
  
}


void SMStateDCMotor(void){
 
  SMState = STATE_DC_MOTOR_MODE; //default next state

  if (doReport) { //flag set in interrupt routine
	report();
	doReport = false; //clear flag so can run again later
  }
  
  
  if (millis() >= mode_start_time + shutdown_timer) {
      SMState = STATE_AFTER_PID_SPEED_MODE;
  }
  
}


void SMStateChangeDCMotorSetPoint(void) {
  
  SMState = STATE_DC_MOTOR_MODE;

  if (abs(DCMotorChangeSpeed) <= DCMotorMaxSpeed) {
	DCMotorSetSpeed = DCMotorChangeSpeed; //leave unchanged if outside range
  }
}

void SMStateAfterDCMotor(void){

  SMState = STATE_AWAITING_STOP;

  
}


void SMStateBeforePIDSpeed(void){

  SMState = STATE_PID_SPEED_MODE; //default next state

  attachInterrupt(digitalPinToInterrupt(encoderPinA), doEncoderASpeed, RISING);


  report_integer = 5;


}


void SMStatePIDSpeed(void){
  
  SMState = STATE_PID_SPEED_MODE; //default next state

  if (doPID) {

	// calculate current speed
	// calculate PID
	// set drive
	
  }

  if (doReport) { //flag set in interrupt routine
	report();
	doReport = false; //clear flag so can run again later
  }

  // It's ok to wait in a state a long time if we are not using the motor
  if (!cmpf(disk.getCommand(),0, speedEpsilon)) {
	if (millis() >= mode_start_time + shutdown_timer) {
	  SMState = STATE_AFTER_PID_SPEED_MODE;
	}
  }
}

void SMStateChangePIDSpeedCoefficients(void){

  SMState = STATE_PID_SPEED_MODE; //default next state
  changePIDCoefficients();
 
}
void SMStateChangePIDSpeedSetPoint(void){

  SMState = STATE_PID_SPEED_MODE; //default next state
  
  if (SMStateNewPIDSpeed >= 0 && SMStateNewPIDSpeed <= maxPIDSpeed) {
 	resetPIDSignal();
    setPointSpeed = SMStateNewPIDSpeed;
   }

}
  

void SMStateAfterPIDSpeed(void){

  SMState = STATE_AWAITING_STOP; //default next state


}



void SMStateBeforePIDPosition(void){

  SMState = STATE_PID_POSITION_MODE; //default next state
  
  report_integer = 1;
  


}


void SMStatePIDPosition(void){

  SMState = STATE_PID_POSITION_MODE; //default next state
  
  if (doPID) {

	// calculate current position
	// calculate PID
	// set drive
	
  }
  
  if (doReport) { //flag set in interrupt routine
	report();
	doReport = false; //clear flag so can run again later
  }
  
  if(stopPIDPosition()){
    SMState = STATE_AFTER_PID_POSITION_MODE;
  }
  // Disk might be oscillating so need to shutdown if it times out
  if (millis() >= mode_start_time + shutdown_timer) {
	  SMState = STATE_AFTER_PID_SPEED_MODE;
  }

}


void SMStateChangePIDPositionSetPoint(void){

  SMState = STATE_PID_POSITION_MODE; //default next state

  if(changePIDPosition >= PIDPositionMin && changePIDPosition <= PIDPositionMax){
	
    pid.setCommand(changePIDPosition); 

  } else{
    Serial.println("{\"err\":\"position setpoint outside range\"}");
  }
}


void SMStateChangePIDPositionCoefficients(void){

  SMState = STATE_PID_POSITION_MODE; //default next state
  changePIDCoefficients();
 
}


void SMStateAfterPIDPosition(void){

  SMState = STATE_AWAITING_STOP;

  detachInterrupt(digitalPinToInterrupt(encoderPinA));
  detachInterrupt(digitalPinToInterrupt(encoderPinB));

}


//*****************************************************************************

//STATE MACHINE RUN FUNCTION
void SMRun(void)
{
  if (SMState < numStates)
  {
    SMState = readSerialJSON(SMState);      // check for incoming commands received
    (*StateMachine[SMState].func)();        //reads the current state and then runs the associated function
    
  }
  else{
    Serial.println("Exception in State Machine");
  }
  
}

//This function is run on a timer interrupt defined by PIDInterval/timer_interrupt_freq.
void TimerInterrupt(void){

  doPID = true;

  report_count++;
  if(report_count >= report_integer){
    doReport = true;     
    report_count = 0;
  }
  
}


//===================================================================================
//====================== SETUP AND LOOP =============================================
//===================================================================================

void setup() {
  //setup encoder pins, pullup resistors on PCB
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  pinMode(indexPin, INPUT); //not using, but just to make sure not set to output as will be connected.

  // leave setting up encoder interrupts to the particular mode which will use them.
   
  //initialise time values
  float t = millis();
  current_time_encoder = t;
  previous_time_encoder = t;
  current_time_index = t;   
  previous_time_index = t;
  mode_start_time = t;

  Serial.setTimeout(50);
  Serial.begin(57600);

  disk.initialise(encoder.read(), micros());
  
  startTimer(timer_interrupt_freq);   //setup and start the timer interrupt functions for PID calculations

  while (! Serial);
}

void loop() {

  // update state machine (which will run tasks flagged by interrupts)
  SM_Run();
  
}

//===================================================================================
//====================== SUPPORTING FUNCTIONS========================================
//
//
//
//            These may be used by one or more states
//
//
//
//
//===================================================================================

//https://noobtuts.com/cpp/compare-float-values
bool cmpf(float a, float b, float epsilon) {
  return (fabs(a - b) < epsilon);
}


void newShutdownTimer(float time){

  if (time == 0) return;

  if ( time <= longestShutdownTimerSeconds){
	shutdownTimerMillis = time * secondsToMillis;
  }
}


void inRange(float val, float min, float max) {
  return (val >= min && val <= max);
}

void changePIDCoefficients(void) {
  
  if (isNewKp) {
	if (inRange(newKp, minKp, maxKp)) {
	  Kp = newKp;
	}
	isNewKp = false;
  }

  if (isNewKi) {
	if (inRange(newKi, minKi, maxKi)) {
	  Ki = newKi;
	}
	isNewKi = false;
  }
  if (isNewKd) {
	if (inRange(newKd, minKd, maxKd)) {
	  Kd = newKd;
	}
	isNewKd = false;
  }
  
  if (isNewTs) {
	if (inRange(newTs, minTs, maxTs)) {
	  PIDInterval = newTs;
	  setTimerFrequency(1000/PIDInterval); 
	}
	isNewTs = false;
  }

  resetPIDSignal();
}



float deadband(float input, float band){
  if(abs(input) <= band){
    return 0.0;
  } else if(input < -band){
    return (input + band);
  } else{
    return (input - band);
  }
}

float friction_compensation_static(float drive_signal, float encoderAngVel, float error){
  //static
  if(abs(encoderAngVel) < 5 && abs(drive_signal) > 0){
	if(error > frictionCompWindow){
	  return frictionCompStaticCCW;
	} else if(error < -frictionCompWindow){
	  return -frictionCompStaticCW;
	} else if(error > 0){
	  return frictionCompStaticCCW * error/frictionCompWindow;
	} else {
	  return -frictionCompStaticCW * error/frictionCompWindow;
	}
  }
  
 
}

//dynamic friction, no window
float friction_compensation_dynamic(float drive_signal, float encoderAngVel, float error){
  if(drive_signal > 0){
      return frictionCompDynamicCCW;
 } else if(drive_signal < 0){
      return -frictionCompDynamicCW;
 } else {
    return 0.0;
  }
 
}

void updateDrivePosition(void) {
  motor.drive(limitDrivePosition(addFC(PID_Signal)));
}

void updateDriveSpeed(void) {
  motor.drive(limitDriveSpeed(addFC(PID_Signal)));
}

float limitDrivePosition(float drive) {
  return limit(drive, minDrivePosition, maxDrivePosition)
}

float limitDriveSpeed(float drive) {
  return limit(drive, minDriveSpeed, maxDriveSpeed)
}

float limit(float val, float min, float max) {

  if(val  > max){
	return max;
  }

  if (val < min) {
	return min
  }

  return val;

}

float addFC(float drive) {

  if (enableFrictionComp) {
	drive +=
	  friction_compensation_static(PID_signal, encoderAngVel, error_speed) +
	  friction_compensation_dynamic(PID_signal, encoderAngVel, error_speed);
  }
  return drive
}

void calculatePID(void){
	if (SMState == STATE_PID_SPEED_MODE){
	  calculateSpeedPID();
	} else if(SMState == STATE_PID_POSITION_MODE){
	  calculatePositionPID();
	}
}

void maybeCalculatePID(void){
  if (doCalculatePID){ //flag set in interrupt routine
	  calculatePID();
	  doCalculatePID = false; // clear flag so can run again later
  }
}




//===================================================================================
//======================  READ AND PARSE JSON COMMMANDS =============================
//
//  This function can and does change the state of the state machine
//
//===================================================================================

StateType readSerialJSON(StateType SMState){
  if(Serial.available() > 0){

	//TODO move to state logic
    mode_start_time = millis();   //on any command sent, reset the start time for hardware switch off
  
    Serial.readBytesUntil(10, command, COMMAND_SIZE);
    deserializeJson(doc, command);
    
    const char* set = doc["set"];

    if(strcmp(set, "speed")==0){
      if(SMState == STATE_PID_SPEED_MODE){
		SMState = STATE_PID_SPEED_CHANGE_SETPOINT;
		changePIDSpeed = doc["to"];
      } else if(SMState == STATE_DC_MOTOR_MODE) {
		SMState = STATE_DC_MOTOR_CHANGE_SETPOINT;
		DCMotorChangeSpeed = doc["to"];
      } else {
		Serial.println("{\"err\":\"in wrong state to set speed\"}");
      }
    }  
    else if(strcmp(set, "position")==0){
      if(SMState == STATE_PID_POSITION_MODE){
		SMState = STATE_CHANGE_PID_POSITION_SETPOINT;
		changePIDPosition = doc["to"];
      } else{
		Serial.println("{\"err\":\"in wrong state to set position\"}");
      }
  } 
    else if(strcmp(set, "mode")==0) {
	  
      const char* new_mode = doc["to"];
	  
      if(SMState == STATE_STOPPED){
        if(strcmp(new_mode, "speedPid") == 0){
          SMState = STATE_BEFORE_PID_SPEED_MODE;
        } 
        else if(strcmp(new_mode, "speedRaw") == 0){
          SMState = STATE_BEFORE_DC_MOTOR_MODE;
        }
        else if(strcmp(new_mode, "positionPid") == 0){
          SMState = STATE_BEFORE_PID_POSITION_MODE;
        }
	  } else {
        if(strcmp(new_mode, "stop") == 0){
          SMState = STATE_AWAITING_STOP;      
        } 
      }
	  
    } else if(strcmp(set, "parameters")==0 || strcmp(set, "p")==0 ) {

      if(!doc["kp"].isNull()){
		newKp = doc["kp"];
		isNewKp = true;
      } else {
		isNewKp = false;
	  }

      if(!doc["ki"].isNull()){
		newKi = doc["ki"];
		isNewKi = true;
      } else {
		isNewKi = false;
	  }

      if(!doc["kd"].isNull()){
		newKd = doc["kd"];
		isNewKd = true;
      } else {
		isNewKd = false;
	  }
	  
      if(!doc["dt"].isNull()){
		newTs = doc["dt"];
		isNewTs = true;
      } else {
		isNewTs = false;
	  }

	  if (isNewKp || isNewKi || isNewKd || isNewTs){
		if ( SMState == STATE_PID_POSITION_MODE) {
		  SMState = STATE_CHANGE_PID_POSITION_COEFFICIENTS;
		} else if (SMState == STATE_PID_SPEED_MODE) {
		  SMState = STATE_CHANGE_PID_SPEED_COEFFICIENTS;
		} else {
		  if (debug) Serial.println("{\"err\":\"in wrong state to set coefficients\"}");
		}
		
	  }
	  
	  if (development) {
		if (!doc["fcs_cw"].isNull()){
		  frictionCompStaticCW = doc["fcs_cw"];
		}
		if (!doc["fcs_ccw"].isNull()){
		  frictionCompStaticCCW = doc["fcs_ccw"];
		}
		if (!doc["fcd_cw"].isNull()){
		  frictionCompDynamicCW = doc["fcd_cw"];
		}
		if (!doc["fcd_ccw"].isNull()){
		  frictionCompDynamicCCW = doc["fcd_ccw"];
		}
		if (!doc["fcw"].isNull()){
		  frictionCompWindow = doc["fcw"];
		}
		if (!doc["pre"].isNull()){
		  motor.setPrescale(doc["pre"]);
		}
	  }
	  
    }
	else if (strcmp(set, "show")==0){
		  
		 const char* new_show = doc["to"];
		  
		 if (strcmp(new_show, "long")==0) {
		   show_mode = SHOW_LONG;
		 } 
		 else if (strcmp(new_show, "short")==0) {
		   show_mode = SHOW_SHORT;
		 } 
		 else if (strcmp(new_show, "shortSpeed")==0) {
		   show_mode = SHOW_SHORT_SPEED;
		 } 
		 else if (strcmp(new_show, "shortPos")==0) {
		   show_mode = SHOW_SHORT_POS;
		 }
		 else if (strcmp(new_show, "plain")==0) {
		   show_mode = SHOW_PLAIN;
		 }
		 else if (strcmp(new_show, "none")==0) {
		   show_mode = SHOW_NONE;
		 }
		 else if (strcmp(new_show, "fc")==0) {
		   show_mode = SHOW_FC;
		 }
    }
    else if(strcmp(set, "timer") == 0){
	  newShutdownTimer(doc["to"]);
     }
  }
      return SMState;     //return whatever state it changed to or maintain the state.
} 


//===================================================================================
//======================  REPORT ====================================================
//
//===================================================================================

void report(void)
{
  // Don't detach interrupts because then we lose counts.
  // Let the outside world handle corrupted messages safely
  
  if (show_mode == SHOW_PLAIN){
    Serial.print("position = ");
    Serial.println(encoderPos);
    Serial.print("ang vel = ");
    Serial.println(encoderAngVel);
  }
  else if (show_mode == SHOW_LONG) {
    Serial.print("{\"enc\":");
    Serial.print(encoderPos);
    Serial.print(",\"enc_ang_vel\":");
    Serial.print(encoderAngVel);
    Serial.print(",\"time\":");
    Serial.print(millis()); 
    Serial.print(",\"p_sig\":");
    Serial.print(proportional_term);
    Serial.print(",\"i_sig\":");
    Serial.print(integral_term);
    Serial.print(",\"d_sig\":");
    Serial.print(derivative_term);
	Serial.print(",\"sp\":");
	Serial.print(setPointPosition);
	Serial.print(",\"sv\":");
	Serial.print(setPointSpeed);
    Serial.println("}");
	
  } else if (show_mode == SHOW_SHORT) {
	Serial.print(millis());
	Serial.print(":");
	Serial.print(setPointPosition);
	Serial.print(":");
	Serial.print(encoderPos);
	Serial.print(":");
	Serial.print(setPointSpeed);
	Serial.print(":");
	Serial.println(encoderAngVel);
	
  } else if (show_mode == SHOW_SHORT_POS) {
	Serial.print(millis());
	Serial.print(",");
	Serial.print(setPointPosition);
	Serial.print(",");
	Serial.println(encoderPos);
	
  } else if (show_mode == SHOW_SHORT_SPEED) {
	Serial.print(millis());
	Serial.print(",");
	Serial.print(setPointSpeed);
	Serial.print(",");
	Serial.println(speed_angular_velocity);
  } else if (show_mode == SHOW_FC) {
	Serial.print(millis());
	Serial.print(",");
	Serial.print(frictionCompStaticCW);
	Serial.print(",");
	Serial.print(frictionCompStaticCCW);
	Serial.print(",");
	Serial.print(frictionCompDynamicCW);
	Serial.print(",");
	Serial.print(frictionCompDynamicCCW);
	Serial.print(",");
	Serial.println(frictionCompWindow);
  }

  else if (show_mode == SHOW_NONE) {
	  // do nothing
  }

}

//===================================================================================
//======================TIMER INTERRUPT FUNCTIONS====================================
//      FROM https://github.com/nebs/arduino-zero-timer-demo/
//===================================================================================

void setTimerFrequency(int frequencyHz) {
  int compareValue = (CPU_HZ / (TIMER_PRESCALER_DIV * frequencyHz)) - 1;
  TcCount16* TC = (TcCount16*) TC3;
  // Make sure the count is in a proportional position to where it was
  // to prevent any jitter or disconnect when changing the compare value.
  TC->COUNT.reg = map(TC->COUNT.reg, 0, TC->CC[0].reg, 0, compareValue);
  TC->CC[0].reg = compareValue;
  while (TC->STATUS.bit.SYNCBUSY == 1);
}

/*
This is a slightly modified version of the timer setup found at:
https://github.com/maxbader/arduino_tools
 */
void startTimer(int frequencyHz) {
  REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID (GCM_TCC2_TC3)) ;
  while ( GCLK->STATUS.bit.SYNCBUSY == 1 );

  TcCount16* TC = (TcCount16*) TC3;

  TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;

  // Use the 16-bit timer
  TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
  while (TC->STATUS.bit.SYNCBUSY == 1);

  // Use match mode so that the timer counter resets when the count matches the compare register
  TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
  while (TC->STATUS.bit.SYNCBUSY == 1);

  // Set prescaler to 1024
  TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024;
  while (TC->STATUS.bit.SYNCBUSY == 1);

  setTimerFrequency(frequencyHz);

  // Enable the compare interrupt
  TC->INTENSET.reg = 0;
  TC->INTENSET.bit.MC0 = 1;

  NVIC_EnableIRQ(TC3_IRQn);

  TC->CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1);
}

void TC3_Handler() {
  TcCount16* TC = (TcCount16*) TC3;
  // If this interrupt is due to the compare register matching the timer count
  // we run the TimerInterrupt() function.
  if (TC->INTFLAG.bit.MC0 == 1) {
    TC->INTFLAG.bit.MC0 = 1;
    TimerInterrupt();           //THE FUNCTION TO RUN ON THE TIMER
  }
}
