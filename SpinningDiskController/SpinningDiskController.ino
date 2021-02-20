
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

// Speed - raw 
int maxSpeedRaw = 100; // percentage TODO check this needed?

float max_rpm = 4000; //TODO check this needed?

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
#define encoderPinA 3     //these pins all have interrupts on them.
#define encoderPinB 2
#define indexPin 11

const float encoderPPR = 500;
const float positionLimit = 2 * encoderPPR; // using both edges on A & B lines for position
const float positionMin = positionLimit * -1; // negative limit
const float positionMax = positionLimit -1;   // assign zero to position half of the rotation, so subtract one

volatile float speed_angular_velocity = 0; //for speed mode velocity reporting
unsigned long speed_current_time_encoder = 0;
unsigned long speed_previous_time_encoder = 0;


/****** INTERRUPTS *******/
// flags for ISR to trigger heavy-weight tasks to run in main loop
volatile bool doReport = false;
volatile bool doPID = false;


/******** PID CONTROL ********/

//PID parameters
float Kp = 1.0;              
float Ki = 0.0;
float Kd = 0.0;

const float minKp = 0;
const float maxKp = 999;
const float minKi = 0;
const float maxKi = 999;
const float minKd = 0;
const float maxKd = 999;
const float minDt = 1;
const float maxDt = 1000;

float PIDInterval = 20.0;       //ms, for timer interrupt 

//pid calculated values
volatile float error = 0;
volatile float previous_error = 0;
volatile float previous_previous_error = 0;
volatile float error_speed = 0;
volatile float previous_error_speed = 0;
volatile float previous_previous_error_speed = 0;
volatile float proportional_term = 0;
volatile float integral_term = 0;
volatile float error_sum = 0;
volatile float derivative_term = 0;
volatile float PID_signal = 0;
volatile float previous_PID_signal = 0;
volatile float previous_previous_PID_signal = 0;

//for both PID modes and error calculations
volatile int encoderPos = 0;
volatile int encoderPosLast = 0;
volatile float encoderAngVel = 0;
volatile int encoder_direction = 1;     //+1 CCW, -1 CW.
volatile int encoder_direction_last = -1;
volatile int encoder_direction_index = 1;
volatile int encoder_positive_count = 0;
volatile int encoder_negative_count = 0;

unsigned long current_time_encoder = 0;
unsigned long previous_time_encoder = 0;

unsigned long current_time_index = 0;
unsigned long previous_time_index = 0;

//lowpass filter
volatile float error_position_filter = 0.0;
volatile float previous_error_position_filter = 0.0;
volatile float previous_previous_error_position_filter = 0.0;
volatile float error_speed_filter = 0.0;
volatile float previous_error_speed_filter = 0.0;
volatile float previous_previous_error_speed_filter = 0.0;

// Position
float setPointPosition = 0; //the position the user has set
float minDrivePosition = -1.0;
float maxDrivePosition = 1.0;

// Speed 
int maxSpeedPID = 200; // limit of the equipment, units rpm
float setPointSpeed = 0;   //user set position for PID_Speed mode and DC_Motor mode
float minDriveSpeed = -1.0;
float maxDriveSpeed = 1.0;

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
int awaiting_stop_lastPos = 0;
int awaiting_stop_thisPos = 0;
int sameCount = 100;   
int sameNeeded = 100;        //number of loops with the same encoder position to assume that motor is stopped.

// SMStateChange<MODE>Coefficients
float newKp = 1.0;
float newKi = 0.0;
float newKd = 0.0;
float newDt = 10;
bool isNewKp = false;
bool isNewKi = false;
bool isNewKd = false;
bool isNewDt = false;

// SMStateChangeDCMotor
float SMStateNewDCMotorSpeed = 0;

// SMStateChangePIDPosition
float SMStateNewPIDPosition = 0;

// SMStateChangePIDSpeed
float SMStateNewPIDSpeed = 0;


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
  STATE_BEFORE_AWAITING_STOP,
  STATE_AWAITING_STOP,  //checking if the motor has stopped
  STATE_STOPPED,        //no drive to motor
  STATE_BEFORE_PID_SPEED_MODE,
  STATE_PID_SPEED_MODE, //pid controller mode - 1st order, speed functions
  STATE_CHANGE_PID_SPEED_SETPOINT,
  STATE_CHANGE_PID_SPEED_COEFFICIENTS,
  STATE_BEFORE_PID_POSITION_MODE,
  STATE_PID_POSITION_MODE,  //pid controller - 2nd order, position functions
  STATE_CHANGE_PID_POSITION_SETPOINT,
  STATE_CHANGE_PID_POSITION_COEFFICIENTS,
  STATE_BEFORE_DC_MOTOR_MODE,
  STATE_DC_MOTOR_MODE,    //regular dc motor functions with no PID controller
  STATE_CHANGE_DC_MOTOR_SETPOINT
} StateType;

//state Machine function prototypes
//these are the functions that run whilst in each respective state.
void SMStateBeforeAwaitingStop(void);
void SMStateAwaitingStop(void);
void SMStateStopped(void);
void SMStateBeforePIDSpeed(void);
void SMStatePIDSpeed(void);
void SMStateChangePIDSpeedSetPoint(void);
void SMStateChangePIDSpeedCoefficients(void);
void SMStateAfterPIDSpeed(void);
void SMStateBeforePosition(void);
void SMStatePIDPosition(void);
void SMStateChangePIDPositionSetPoint(void);
void SMStateChangePIDPositionCoefficients(void);
void SMStateAfterPIDPosition(void);
void SMStateBeforeDCMotor(void);
void SMStateDCMotor(void);
void SMStateChangeDCMotorSetPoint(void);
void SMStateAfterDCMotor(void);
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
  {STATE_BEFORE_AWAITING_STOP, SMStateBeforeAwaitingStop},
  {STATE_AWAITING_STOP, SMStateAwaitingStop},
  {STATE_STOPPED, SMStateStopped},
  {STATE_BEFORE_PID_SPEED_MODE, SMStateBeforePIDSpeed}, 
  {STATE_PID_SPEED_MODE, SMStatePIDSpeed},
  {STATE_CHANGE_PID_SPEED_SETPOINT, SMStateChangePIDSpeedSetPoint},
  {STATE_CHANGE_PID_SPEED_COEFFICIENTS, SMStateChangePIDSpeedCoefficients},  
  {STATE_AFTER_PID_SPEED_MODE, SMStateAfterPIDSpeed},
  {STATE_BEFORE_PID_POSITION_MODE, SMStateBeforePIDPosition},  
  {STATE_PID_POSITION_MODE, SMStatePIDPosition},
  {STATE_CHANGE_PID_POSITION_SETPOINT, SMStateChangePIDPositionSetPoint},
  {STATE_CHANGE_PID_POSITION_COEFFICIENTS, SMStateChangePIDPositionCoefficients},  
  {STATE_PID_AFTER_POSITION_MODE, SMStateAfterPIDPosition},
  {STATE_BEFORE_DC_MOTOR_MODE, SMStateBeforeDCMotor},  
  {STATE_DC_MOTOR_MODE, SMStateDCMotor}
  {STATE_CHANGE_DC_MOTOR_MODE, SMStateChangeDCMotor}
  {STATE_AFTER_DC_MOTOR_SETPOINT, SMStateAfterDCMotorSetPoint}
};
 
int numStates = 17;

/**
 * Stores the current state of the state machine
 */
 
StateType SMState = STATE_STOPPED;    //START IN THE STOPPED STATE

//DEFINE STATE MACHINE FUNCTIONS================================================================

void SMStateStopped(void){        

  SMState = STATE_STOPPED; //default next state

  resetPIDSignal();
 
  setPointPosition = encoderPos;
  setPointSpeed = 0;
  encoderAngVel = 0;

}

void SMStateBeforeAwaitingStop(void){

  SMState = STATE_AWAITING_STOP; //default next state
 
  resetPIDSignal();
 
}

void SMStateAwaitingStop(void){

  //TODO - consider using PID speed mode to stop faster.

  SMState = STATE_AWAITING_STOP; //default next state
 
  motor.brake();

  awaiting_stop_lastPos = awaiting_stop_thisPos;
  awaiting_stop_thisPos = encoderPos;

  if (awaiting_stop_thisPos == awaiting_stop_lastPos){
	sameCount -= 1;
  }
  else{
	sameCount = sameNeeded;
  }
  
  if (sameCount <= 0){
	sameCount = sameNeeded; // avoid edge case on next stop event where enter stop mode at moment direction is reversing and encoder momentarily same value
	SMState = STATE_STOPPED;   
  } 
}


void SMStateBeforeDCMotor(void){

  SMState = STATE_DC_MOTOR_MODE; //default next state
  
  attachInterrupt(digitalPinToInterrupt(encoderPinA), doEncoderASpeed, RISING);
  previous_time_encoder = micros();

  report_integer = 5;
  
}


void SMStateDCMotor(void){
 
  SMState = STATE_DC_MOTOR_MODE; //default next state

  if(stopRawSpeed()){
    SMState = STATE_AWAITING_STOP;
  }
}


void SMStateChangeDCMotorSetPoint(void) {
  
  SMState = STATE_DC_MOTOR_MODE;

  if (abs(SMStateNewRawSpeed) <= maxRawSpeed) {
	setPointSpeed =SMStateNewRawSpeed; //leave unchanged if outside range
  }
}

void SMStateAfterDCMotor(void){

  SMState = STATE_AWAITING_STOP;
  
  detachInterrupt(digitalPinToInterrupt(encoderPinA));
  
}


void SMStateBeforePIDSpeed(void){

  SMState = STATE_PID_SPEED_MODE; //default next state

  attachInterrupt(digitalPinToInterrupt(encoderPinA), doEncoderASpeed, RISING);
  previous_time_encoder = micros();

  report_integer = 5;

  resetPIDSignal();

}


void SMStatePIDSpeed(void){
  
  SMState = STATE_PID_SPEED_MODE; //default next state

  if (doPID) {

	// calculate current speed
	// calculate PID
	// set drive
	
  }

  if(stopPIDSpeed()){
    SMState = STATE_AFTER_PID_SPEED_MODE;
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
  detachInterrupt(digitalPinToInterrupt(encoderPinA));

}



void SMStateBeforePIDPosition(void){

  SMState = STATE_PID_POSITION_MODE; //default next state
  
  report_integer = 1;
  
  attachInterrupt(digitalPinToInterrupt(encoderPinA), doEncoderAPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), doEncoderBPosition, CHANGE);
  previous_time_encoder = micros();

  resetPIDSignal();

}


void SMStatePIDPosition(void){

  SMState = STATE_PID_POSITION_MODE; //default next state
  
  if (doPID) {

	// calculate current position
	// calculate PID
	// set drive
	
  }
  if(stopPIDPosition()){
    SMState = STATE_AFTER_PID_POSITION_MODE;
  }
}


void SMStateChangePIDPositionSetPoint(void){

  SMState = STATE_PID_POSITION_MODE; //default next state

  if(new_position >= positionMin && new_position <= positionMax){
    resetPIDSignal();
    setPointPosition = new_position; //TODO rename setPointPosition
    
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
  
  startTimer(timer_interrupt_freq);   //setup and start the timer interrupt functions for PID calculations

  while (! Serial);
}

void loop() {

  // do tasks flagged for action by interrupt routines
  maybeReport()
 
  // update state machine
  SM_Run();
  
}

//===================================================================================
//====================== SUPPORTING FUNCTIONS========================================
//===================================================================================

bool stopPIDSpeed(void) {
  return (millis() >= mode_start_time + shutdown_timer) && (setPointSpeed != 0);
}

bool stopRawSpeed(void) {
  return stopPIDSpeed();
}

bool stopPIDPosition(void) {
  return millis() >= mode_start_time + shutdown_timer;
}


void newShutdownTimer(float time){

  if (time == 0) return;

  if ( time <= longestShutdownTimerSeconds){
	shutdownTimerMillis = time * secondsToMillis;
  }
}



// Interrupt on encoder A changing state
// calculate position, direction and speed
void doEncoderAPosition() {
  previous_time_encoder = current_time_encoder;
  current_time_encoder = micros();

  bool A_set = digitalRead(encoderPinA) == HIGH;
  bool B_set = digitalRead(encoderPinB) == HIGH;
  
  // Idiosyncractically, a negative encoder direction
  // equates to positive count increase
  // TODO - make this consistent, and fix all the maths that relies on it

  encoder_direction = ((A_set != B_set) ? -1 : +1);
  
  encoderPos -= encoder_direction;

 
  unsigned long dt = current_time_encoder - previous_time_encoder; 

  if (dt > 0 ) { //not overflow
	encoderAngVel = encoder_direction * 60e6 / (dt * encoderPPR);
  }
  
  if (encoderPos == 0) { // we can't skip this because increments are by one
	speed_current_time_encoder = micros();
	unsigned long speed_dt = speed_current_time_encoder - speed_previous_time_encoder;
	
	if (speed_dt > 0 ) { //not overflow
	  speed_angular_velocity =  encoder_direction * 60e6 / (speed_dt); //rpm 
	  }

	if (debug) {
	  Serial.print("{\"enc_ang_vel\":");
	  Serial.print(encoderAngVel);
	  Serial.print(",\"time\":");
	  Serial.print(millis()); 
	  Serial.println("}");  
	}
	
	speed_previous_time_encoder = speed_current_time_encoder; 
  }
}

// interrupt on A rising edge only
void doEncoderASpeed() {

  bool A_set = digitalRead(encoderPinA) == HIGH;
  bool B_set = digitalRead(encoderPinB) == HIGH;
  
  // Idiosyncractically, a negative encoder direction
  // equates to positive count increase
  // TODO - make this consistent, and fix all the maths that relies on it

  encoder_direction = ((A_set != B_set) ? -1 : +1);
  encoderPos -= encoder_direction;
  
  if (encoderPos == 0) { 
	speed_current_time_encoder = micros();
	unsigned long speed_dt = speed_current_time_encoder - speed_previous_time_encoder;
	
	if (speed_dt > 0 ) { //not overflow
	  speed_angular_velocity =  encoder_direction * 60e6 / (speed_dt); //rpm 
	  }

	if (debug) {
	  Serial.print("{\"enc_ang_vel\":");
	  Serial.print(encoderAngVel);
	  Serial.print(",\"time\":");
	  Serial.print(millis()); 
	  Serial.println("}");  
	}
	
	speed_previous_time_encoder = speed_current_time_encoder; 
  }
}


// Interrupt on B changing state
void doEncoderBPosition() {
  
  bool A_set = digitalRead(encoderPinA) == HIGH;
  bool B_set = digitalRead(encoderPinB) == HIGH;

  encoder_direction = ((A_set != B_set) ? -1 : +1);
  encoderPos -= encoder_direction;

  encoderWrap();

}

void encoderWrap(void){
  if (encoderPos >= position_limit) { // top positive limit can't be reached
    encoderWrapCW++;
    encoderWrapCCW = 0;
    encoderPos -= 2*position_limit; // so turn it into bottom limit
  } else if (encoderPos < -position_limit) { //bottom positive limit CAN be reached
	encoderWrapCCW++;
	encoderWrapCW = 0;
	encoderPos += 2*position_limit; 
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
  
  if (isNewDt) {
	if (inRange(newDt, minDt, maxDt)) {
	  PIDInterval = newDt;
	  setTimerFrequency(1000/PIDInterval); 
	}
	isNewDt = false;
  }

  resetPIDSignal();
}

void resetPIDSignal(void){
  PID_signal = 0;
  previous_PID_signal = 0;
  previous_previous_PID_signal = 0;

  error = 0;
  previous_error = 0;
  previous_previous_error = 0;
  error_speed = 0;
  previous_error_speed = 0;
  previous_previous_error_speed = 0;

  error_speed_filter = 0;
  previous_error_speed_filter = 0;
  previous_previous_error_speed_filter = 0;

  error_position_filter = 0;
  previous_error_position_filter = 0;
  previous_previous_error_position_filter = 0;

  proportional_term = 0.0;
  integral_term = 0.0;
  derivative_term = 0.0;
  
}


//DISCRETE TIME VERSION, with filter
void calculateSpeedPID(void){
    previous_previous_error_speed = previous_error_speed;
    previous_error_speed = error_speed;

    previous_previous_error_speed_filter = previous_error_speed_filter;
    previous_error_speed_filter = error_speed_filter;

    error_speed = (setPointSpeed - encoderAngVel)/100.0;

    error_speed_filter = lowpass_filter(error_speed, previous_error_speed_filter);
  
  float delta_t = PIDInterval/1000.0;
  float Ti = Kp/Ki;
  float Td = Kd/Kp;

  float delta_p = Kp*(error_speed - previous_error_speed);
  proportional_term += delta_p;

  float delta_i = Kp*delta_t*error_speed/Ti;
  integral_term += delta_i;

   float delta_d = Kp*(Td/delta_t)*(error_speed_filter - 2*previous_error_speed_filter + previous_previous_error_speed_filter);
  derivative_term += delta_d; 

  
 //float new_signal = Kp*(error_speed - previous_error_speed + delta_t*error_speed/Ti +(Td/delta_t)*(error_speed_filter - 2*previous_error_speed_filter + previous_previous_error_speed_filter));

  float new_signal = delta_p + delta_i + delta_d;
  
  PID_signal += new_signal;
    
}


////DISCRETE TIME VERSION with filter
void calculatePositionPID(void){
    previous_previous_error = previous_error;
    previous_error = error;

    previous_previous_error_position_filter = previous_error_position_filter;
    previous_error_position_filter = error_position_filter;
    
  float error_pos = encoderPos - setPointPosition;
  int dir = error_pos / abs(error_pos);    //should be +1 or -1.
  
  float error_pos_inverse = 2*positionLimit - abs(error_pos);

  if(abs(error_pos) <= abs(error_pos_inverse)){
    error = error_pos;
  } else {
    error = -1*dir*error_pos_inverse;
  }
  //convert error to an angular error in deg
  error = error*180.0/positionLimit;

  error_position_filter = lowpass_filter(error, previous_error_position_filter);
  
  
  float delta_t = PIDInterval/1000.0;
  float Ti = Kp/Ki;
  float Td = Kd/Kp;

  float delta_p = Kp*(error - previous_error);
  proportional_term += delta_p;

  float delta_i = Kp*delta_t*error/Ti;
  integral_term += delta_i;

  float delta_d =  Kp*(Td/delta_t)*(error_position_filter - 2*previous_error_position_filter + previous_previous_error_position_filter);
  derivative_term += delta_d; 
  
  float new_signal = delta_p + delta_i + delta_d;
  
  PID_signal += new_signal;
    

}


// TODO consider the delay that (any!) filter introduces
// Plant will low pass filter to a certain extent.
float lowpass_filter(float input, float previous_output){
  float a = 0.1;
  return (1-a)*previous_output + a*input;
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

void maybeReport(void){
  if (doReport) { //flag set in interrupt routine
	report();
	doReport = false; //clear flag so can run again later
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
		SMState = STATE_CHANGE_PID_SPEED_SETPOINT;
		SMStateNewPIDSpeed = doc["to"];
      } else if(SMState == STATE_DC_MOTOR_MODE) {
		SMState = STATE_CHANGE_DC_MOTOR_MODE;
		SMStateNewRawSpeed = doc["to"];
      } else {
		Serial.println("{\"err\":\"in wrong state to set speed\"}");
      }
    }  
    else if(strcmp(set, "position")==0){
      if(SMState == STATE_PID_POSITION_MODE){
		SMState = STATE_CHANGE_PID_POSITION_SETPOINT;
		SMStateNewPIDPosition = doc["to"];
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
		newDt = doc["dt"];
		isNewDt = true;
      } else {
		isNewDt = false;
	  }

	  if (isNewKp || isNewKi || isNewKd || isNewDt){
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
