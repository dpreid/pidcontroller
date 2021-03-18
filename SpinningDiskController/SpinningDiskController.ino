
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
bool trace = false;
bool permitOverspeed = false;

//=============================================================

/********** HEADERS ******************/
#include "ArduinoJson-v6.9.1.h"
#include <dcmotor.h>
#include <Encoder.h>
#include <MotorControllerBTS7960.h>
#include <pid.h>
#include <rotaryPlant.h>

/******** PID CONTROL ********/

//PID parameters
float Kp = 1.0;
float Ki = 0.0;
float Kd = 0.0;
float Ts = 0.005;
float N = 5;
float uMin = -1;
float uMax = +1;

float positionPIDScaleFactor = 0.4124;

PID controller = PID(Kp,Ki,Kd,Ts,N,uMin,uMax);

const float KpMin = 0;
const float KpMax = 99;
const float KiMin = 0;
const float KiMax = 99;
const float KdMin = 0;
const float KdMax = 99;
const float TsMin = 0.002; //2 milliseconds
const float TsMax = 0.100; //100 milliseconds


/******** DC MOTOR ******/


// Pins connecting to drive board
#define AIN1 5 //old DIRECTION
#define enablePin  6 //old PWMA ENABLE
#define leftPWMPin 4
#define rightPWMPin 7

const int direction = 1; // If motor spins in the opposite direction then you can change this to -1.

// MotorBTS7960(int PWMA, int leftPWMPin, int rightPWMPin, int timerNumber, int offset, long prescale)
//timer0 for pin6, timer1 for 4&7 https://github.com/ocrdu/Arduino_SAMD21_turbo_PWM, was 960000 for 48MHz/960000=50Hz
MotorBTS7960 motor = MotorBTS7960(enablePin, leftPWMPin, rightPWMPin, 1, direction, 24000); // 2 kHz
//96000 for 500Hz, 120000 for 400Hz, 240000 for 200Hz PWM, 480000 for 100Hz PWM, 960000 for 50Hz


/******* Drive signals ********/

// MOTOR
static float plantForMotor[] = {-12,12}; //+/- 100% in the app
static float driveForMotor[] = {-1,1}; // was 0.4 
static int sizeMotor = 2;
Driver driverMotor = Driver(plantForMotor, driveForMotor, sizeMotor);
float motorPrimaryOffsetPos = 0; //set in setup()
float motorPrimaryOffsetNeg = 0; //set in setup()

// POSITION
static float plantForPosition[] = {-1,+1}; //was 1 
static float driveForPosition[] = {-1,1}; // max 50% drive
static int sizePosition = 2;
Driver driverPosition = Driver(plantForPosition, driveForPosition, sizePosition);
float positionPrimaryOffsetPos = 0.0525; //set in setup()
float positionPrimaryOffsetNeg = -0.0525;  //set in setup()


// VELOCITY

float velocityLimit = 80; // in rps just over give a buffer so we can PID up to velocityMaxRPS, plus so we can reason separately about
                      // error-to-drive mapping, and safe operating limits.
float velocityMaxRPS = 32; // 16rps is 960 rpm we can probably get to ~2500 rpm if we risk the bearings
float plantMaxDifference = 8;
static float plantForVelocity[] = {-plantMaxDifference, plantMaxDifference}; //+/- 100% in the app
static float driveForVelocity[] = {-1,1}; // max 50% drive
static int sizeVelocity = 2;
Driver driverVelocity = Driver(plantForVelocity, driveForVelocity, sizeVelocity);
float velocityPrimaryOffsetPos = 0.0525; //set in setup()
float velocityPrimaryOffsetNeg = -0.0525;  //set in setup()


// Timer - to switch off motor at end of a run
float lastCommandMillis = 0;
float longestShutdownTimeMillis = 180 * 1000;
float shutdownTimeMillis = 0.5 * longestShutdownTimeMillis;


/****** ENCODER *******/

#define encoderPinA 3     //these pins all have interrupts on them.
#define encoderPinB 2
#define indexPin 11

const float encoderPPR = 2000;
const float LPFCoefficient = 0.9;

RotaryPlant disk = RotaryPlant(encoderPPR, LPFCoefficient, Ts);

const float positionLimit = 2 * encoderPPR; // using both edges on A & B lines for position
const float encoderMin = positionLimit * -1; // negative limit
const float encoderMax = positionLimit -1;   // assign zero to position half of the rotation, so subtract one
const float positionPlantMin  = -5; // limits for controller 
const float positionPlantMax = 5;
const float positionLimitMin  = -2; // limits for state to enforce
const float positionLimitMax = 2;

volatile int velocityLimitCount = 0;
volatile int positionLimitCount = 0;
const int velocityLimitCountThreshold = 10;
const int positionLimitCountThreshold = 10;

volatile float velocity_angular_velocity = 0; //for velocity mode velocity reporting
unsigned long velocity_current_time_encoder = 0;
unsigned long velocity_previous_time_encoder = 0;

const float positionEpsilon = 1.0f / (float)encoderPPR;
const float velocityEpsilon = positionEpsilon; //used to check if command is close to zero, value not critical.

/****** INTERRUPTS *******/
// flags for ISR to trigger heavy-weight tasks to run in main loop
volatile bool doReport = false;
volatile bool doPID = false;
volatile bool updated = false;
// flags for ISR to action
volatile bool requestZeroPosition; 


float PIDInterval = Ts * 1000;  //ms, for timer interrupt

// Timer
#define CPU_HZ 48000000
#define TIMER_PRESCALER_DIV 1024
float timer_interrupt_freq = 1000.0/PIDInterval;


/******* SERIAL COMMANDS *******/


//JSON serialization
#define COMMAND_SIZE 128  // was 64
StaticJsonDocument<COMMAND_SIZE> doc;
char command[COMMAND_SIZE];
#define REPORT_SIZE 128 
char writeBuffer[REPORT_SIZE];

// Data reporting format
#define SHOW_NONE 0
#define SHOW_SHORT 1
#define SHOW_LONG 2
#define SHOW_FC 3

volatile int show_mode = SHOW_LONG;

int reportVelocity = 20;
int reportPosition = 4;
int report_integer = 1;          //an integer multiple of the PIDInterval for reporting data set to 5 for velocity modes, 1 for position mode.
int report_count = 0;

/******** OTHER GLOBAL VARIABLES ********/
volatile long pulse = 0;
volatile long count = 0; //count
volatile long lastCount = 0;//
long offset = 0; //

bool initialiseDisk;

int loop_count = 0; //check if needed?
unsigned long dta; // moving average, needs right shifting by 3 bits to get correct value

bool writing = false; //semaphore for coordinating writes to serial port
int apiVersion = 0; //legacy version
long reportCount = 0; //for legacy report frequency

float motorDriveVolts = 0;
float motorMaxVolts = 12;

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
float newN = 2;
bool isNewKp = false;
bool isNewKi = false;
bool isNewKd = false;
bool isNewTs = false;
bool isNewN = false;

// SMStateChangeDCMotor
float motorMaxCommand = 10.0; //external world units are 0 to +/-12V
float motorMinCommand = 0.0; //we're not actually using this yet ....
float motorChangeCommand = 0;
float motorCommand = 0;

// SMStateChangePIDPosition
float positionChangeCommand = 0;
float positionCommandMin = -2;  //wider range, to permit ramping
float positionCommandMax = +2;
 

// SMStateChangePIDVelocity
float velocityChangeCommand = 0;
float velocityCommandMin = -200; //rad/sec
float velocityCommandMax = +200; //rps

// TODO sort out normalisation of velocity command based on defined plant maximum
// TODO consider linearising the drive to bring constant-gain regime down to slower
// velocitys which are less stressful on the bearings due to the wobble


//=============================================================
// Function Prototypes
//=============================================================

void newShutdownTimer(float time);
bool inRange(float val, float min, float max);
void changePIDCoefficients(void);
void startTimer(int frequencyHz);
void setTimerFrequencyHz(int frequencyHz);
void requestSerial(void);
void releaseSerial(void);


/**
 * Defines the valid states for the state machine
 *
 */
typedef enum
{
  STATE_STOPPING_BEFORE,
  STATE_STOPPING_DURING,  
  STATE_STOPPING_AFTER,          
  STATE_STOPPED,          
  STATE_MOTOR_BEFORE,
  STATE_MOTOR_DURING, 
  STATE_MOTOR_CHANGE_COMMAND,
  STATE_MOTOR_CHANGE_PARAMETERS,
  STATE_MOTOR_AFTER,
  STATE_VELOCITY_BEFORE,
  STATE_VELOCITY_DURING, 
  STATE_VELOCITY_CHANGE_COMMAND,
  STATE_VELOCITY_CHANGE_PARAMETERS,
  STATE_VELOCITY_AFTER,
  STATE_POSITION_BEFORE,
  STATE_POSITION_WAITING,
  STATE_POSITION_READY,
  STATE_POSITION_DURING, 
  STATE_POSITION_CHANGE_COMMAND,
  STATE_POSITION_CHANGE_PARAMETERS,
  STATE_POSITION_AFTER,
} StateType;

//state Machine function prototypes
//these are the functions that run whilst in each respective state.
void stateStoppingBefore(void);
void stateStoppingDuring(void);
void stateStoppingAfter(void);
void stateStopped(void);
void stateMotorBefore(void);
void stateMotorDuring(void);
void stateMotorChangeCommand(void);
void stateMotorChangeParameters(void);
void stateMotorAfter(void);
void stateVelocityBefore(void);
void stateVelocityDuring(void);
void stateVelocityChangeCommand(void);
void stateVelocityChangeParameters(void);
void stateVelocityAfter(void);
void statePositionBefore(void);
void statePositionWaiting(void);
void statePositionReady(void);
void statePositionDuring(void);
void statePositionChangeCommand(void);
void statePositionChangeParameters(void);
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
  {STATE_STOPPING_BEFORE,            stateStoppingBefore},
  {STATE_STOPPING_DURING,            stateStoppingDuring},
  {STATE_STOPPING_AFTER,             stateStoppingAfter},
  {STATE_STOPPED,                    stateStopped},
  {STATE_MOTOR_BEFORE,               stateMotorBefore},
  {STATE_MOTOR_DURING,               stateMotorDuring},
  {STATE_MOTOR_CHANGE_COMMAND,       stateMotorChangeCommand},
  {STATE_MOTOR_CHANGE_PARAMETERS,    stateMotorChangeParameters},
  {STATE_MOTOR_AFTER,                stateMotorAfter},
  {STATE_VELOCITY_BEFORE,            stateVelocityBefore},
  {STATE_VELOCITY_DURING,            stateVelocityDuring},
  {STATE_VELOCITY_CHANGE_COMMAND,    stateVelocityChangeCommand},
  {STATE_VELOCITY_CHANGE_PARAMETERS, stateVelocityChangeParameters},
  {STATE_VELOCITY_AFTER,             stateVelocityAfter},
  {STATE_POSITION_BEFORE,            statePositionBefore},
  {STATE_POSITION_WAITING,           statePositionWaiting},
  {STATE_POSITION_READY,             statePositionReady},  
  {STATE_POSITION_DURING,            statePositionDuring},
  {STATE_POSITION_CHANGE_COMMAND,    statePositionChangeCommand},
  {STATE_POSITION_CHANGE_PARAMETERS, statePositionChangeParameters},
  {STATE_POSITION_AFTER,             statePositionAfter},
};

int numStates = 21;

/**
 * Stores the current state of the state machine
 */

StateType state = STATE_STOPPED;    //START IN THE STOPPED STATE

//============================================================================
//           DEFINE STATE MACHINE FUNCTIONS
//
// A typical sequence uses the BEFORE state as the entry point for a given task:
//
//    STATE_<STATE>_BEFORE
//    STATE_<STATE>_DURING (repeat until finished)
//    STATE_<STATE>_AFTER
//
//  Where not currently required, these states are provided as placeholders
//  for ease of future modification.
//
//  If a change to parameters or command is needed, the typical sequence becomes 
//
//    STATE_<STATE>_BEFORE
//    STATE_<STATE>_DURING (repeat until change)
//    STATE_<STATE>_CHANGE_<SOMETHING>
//    STATE_<STATE>_DURING (repeat until finished)
//    STATE_<STATE>_AFTER
//
//  Note that you should NOT go back to BEFORE following a change -
//  reimplement any required logic from BEFORE in CHANGE_<SOMETHING>
//  so that truly one-shot setup stuff can be put in BEFORE.
// 
//  The default NEXT state is set in the first line of each state,
//  then overridden if need be.
//
//  An exception to this convention are the additional STATE_POSITION_WAITING,
//  and STATE_POSITION_READY, which are intended to let you set the PID parameters
//  before running the PID control routine, so that users can recover from setting
//  large integral coefficients, which cause an automatic stop for going out
//  of bounds before a lower PID value can be set. On entering position PID mode
//  the before state zeroes, the disk, then passes to the READY state.
//
//============================================================================

void stateStoppingBefore(void) {

  state = STATE_STOPPING_DURING;

  // deliberately empty state

}

void stateStoppingDuring(void) {

  state = STATE_STOPPING_DURING; 

  motor.brake(); //This is just free-wheeling, so needs no further commands.
  motorDriveVolts = 0;
  
  awaitingStopP1 = awaitingStopP0;
  awaitingStopP0 = disk.getDisplacement();

  if (cmpf(awaitingStopP0, awaitingStopP1, positionEpsilon)) {
    awaitingStopCount -= 1;
  }
  else {
    awaitingStopCount = awaitingStopNeeded;
  }

  if (doReport) { //flag set in interrupt routine
    report();
    doReport = false; //clear flag so can run again later
  }
  
  if (awaitingStopCount <= 0) {
	// preload awaitingStopCount to handle edge case of triggering stop
	// right as disk changes direction, giving two idential readings.
    awaitingStopCount = awaitingStopNeeded; 
    state = STATE_STOPPING_AFTER;
  }
}

void stateStoppingAfter(void) {

  state = STATE_STOPPED;

  // deliberately empty state

}


void stateStopped(void) {

  state = STATE_STOPPED; //default next state
  if (doReport) { //flag set in interrupt routine
    report();
    doReport = false; //clear flag so can run again later
  }

}

void stateMotorBefore(void) {

  state = STATE_MOTOR_DURING;

  lastCommandMillis = millis();

  velocityLimitCount = 0;
  
  motorChangeCommand = 0; 
  motorCommand = 0; //start with motor off

  report_integer = reportVelocity;

}


void stateMotorDuring(void) {

  state = STATE_MOTOR_DURING;

  // Do nothing to motor drive, as already set

  if (doReport) { //flag set in interrupt routine
    report();
    doReport = false; //clear flag so can run again later
  }

  if (abs(disk.getVelocity()) > velocityLimit) {
	velocityLimitCount++;
  } else {
	velocityLimitCount = 0;
  }

  if (velocityLimitCount > velocityLimitCountThreshold) {
	state = STATE_MOTOR_AFTER;
	Serial.println("{\"error\":\"velocity limit exceeded\"}");
	Serial.println("{​\"error\":\"limit\",\"type\":\"velocity\",\"state\":\"stopping\"}");
  }
 
  
  if (millis() >= lastCommandMillis + shutdownTimeMillis) {
	Serial.println("{\"warn\":\"maximum run time exceeded\"}");
	Serial.println("{​\"error\":\"limit\",\"type\":\"time\",\"state\":\"stopping\"}");
    state = STATE_MOTOR_AFTER;
  }

}


void stateMotorChangeCommand(void) {

  state = STATE_MOTOR_DURING;

  lastCommandMillis = millis();
  
  if (debug) Serial.println("StateMotorChangeCommand");
  
  if (abs(motorChangeCommand) <= motorMaxCommand) {
	
    motorCommand = motorChangeCommand; //leave unchanged if outside range

	float yp = driverMotor.drive(motorCommand, 0.0);
	
	motor.drive(yp); //driverMotors handles conversion from +/-100% to +/-1.0
	
	motorDriveVolts = yp * motorMaxVolts;
	
	if (debug) {
	  Serial.print("Changed motorCommand to ");
	  Serial.println(motorCommand);
	}
  } else {
	Serial.println("{\"error\":\"cannot command voltage outside range\"}");
	Serial.println("{​\"error\":\"command\",\"type\":\"limit\",\"state\":\"motor\"}");
  }
  
}
void stateMotorChangeParameters(void) {

  state = STATE_MOTOR_DURING;

  lastCommandMillis = millis();
  
  // deliberately empty state
  
}

void stateMotorAfter(void) {

  state = STATE_STOPPING_BEFORE;

  motorChangeCommand = 0;
  
  motorCommand = 0;

  motor.brake(); 
  motorDriveVolts = 0;
  
  if (debug) Serial.println("{\"state\":\"MotorAfter\"}");
}


void stateVelocityBefore(void) {

  state = STATE_VELOCITY_DURING;

  lastCommandMillis = millis();

  velocityLimitCount = 0;
  
  velocityChangeCommand = 0;

  controller.setLimits(-velocityMaxRPS,velocityMaxRPS);
  controller.setCommand(velocityChangeCommand);

  report_integer = reportVelocity;
}


void stateVelocityDuring(void) {

  state = STATE_VELOCITY_DURING; 

  float v, y, yp;
  float c;
  
  if (doPID) {
    c = controller.getCommand();
    v = disk.getVelocity();
    
	y = controller.update(v);

	yp = driverVelocity.drive(y,v);
	
	motor.drive(yp);
	motorDriveVolts = yp * motorMaxVolts;
  }

  if (doReport) { //flag set in interrupt routine
    report();
	if (debug) {
	  Serial.print("c=");
	  Serial.print(controller.getCommand());
	  Serial.print(", v=");
	  Serial.print(v);
	  Serial.print(", error=");
	  Serial.print(c-v);
	  Serial.print(", y=");	  
	  Serial.print(y);
	  Serial.print(", e0=");	  
	  Serial.print(controller.getError());	  
	  Serial.print(", *yp=");
	  Serial.print(yp);
	  Serial.print(", Kp=");
	  Serial.print(controller.getKp());
	  Serial.print(", Ki=");
	  Serial.print(controller.getKi());	  
	  Serial.print(", Kd=");
	  Serial.print(controller.getKd());
	  Serial.print(", Ts=");
	  Serial.print(controller.getTs());
	  Serial.print(", N=");
	  Serial.println(controller.getN());	  
	}
	
    doReport = false; //clear flag so can run again later
  }

  if (abs(disk.getVelocity()) > velocityLimit) {
	velocityLimitCount++;
  } else {
	velocityLimitCount = 0;
  }

  if (velocityLimitCount > velocityLimitCountThreshold) {
	state = STATE_MOTOR_AFTER;
	Serial.println("{\"error\":\"velocity limit exceeded\"}");
	Serial.println("{​\"error\":\"limit\",\"type\":\"velocity\",\"state\":\"stopping\"}");
  }
  
  
  // It's ok to wait in a state a long time if we are NOT using the motor
  if (!cmpf(controller.getCommand(),0, velocityEpsilon)) {
    if (millis() >= lastCommandMillis + shutdownTimeMillis) {
	  Serial.println("{\"warn\":\"maximum run time exceeded\"}");
	  Serial.println("{​\"error\":\"limit\",\"type\":\"time\",\"state\":\"stopping\"}");
      state = STATE_VELOCITY_AFTER;
    }
  }
}

void stateVelocityChangeCommand(void) {

  state = STATE_VELOCITY_DURING;

  lastCommandMillis = millis();

  if (abs(velocityChangeCommand) <= velocityCommandMax) {
	controller.setCommand(velocityChangeCommand);
	Serial.print("{\"info\":\"new velocity command\",\"c\":\"");
	Serial.print(velocityToExternalUnits(controller.getCommand()));
	Serial.println("\"}"); 
  } else {
	Serial.println("{\"error\":\"cannot command velocity outside range\"}");
	Serial.println("{​\"error\":\"command\",\"type\":\"limit\",\"state\":\"velocity\"}");
  }
}


void stateVelocityChangeParameters(void) {

  state = STATE_VELOCITY_DURING;

  lastCommandMillis = millis();
  
  changePIDCoefficients();

}



void stateVelocityAfter(void) {

  state = STATE_STOPPING_BEFORE;
  
  // deliberately empty state
			 

}



void statePositionBefore(void) {

  state = STATE_POSITION_BEFORE;

  lastCommandMillis = millis();

  velocityLimitCount = 0;
  positionLimitCount = 0;

  float p = disk.getDisplacement();

  if ( abs(p) <= positionEpsilon) {

	requestZeroPosition = false;

	controller.setLimits(positionPlantMin, positionPlantMax);

	positionChangeCommand = 0;
	controller.setCommand(positionChangeCommand);

	report_integer = reportPosition;

	//scale Kp during position mode
	controller.setKp(controller.getKp() * positionPIDScaleFactor);

	state = STATE_POSITION_READY; // go to the READY state to await command or parameter change
	
	if (debug) Serial.println("Disk zeroed");
	
  } else {

	requestZeroPosition = true;

	if (debug) {
	  requestSerial();
	  Serial.print("Disk not zeroed, p = ");
	  Serial.println(p);
	  releaseSerial();
	}
  
  }

}


void statePositionWaiting(void) {

  state = STATE_POSITION_WAITING;

  // since we from DURING,where the disk can be spinning,
  // we need to duplicate some commands from the stopping and before states
  // to ensure the disk is stopped and zero'd. We don't want to just use the
  // before state, because this one-shot setup (like adjusting the Kp value)
  // which cannot be done more than once per entry into the mode.
  // So we do the disk stopping we need here.
   
  motor.brake(); //This is just free-wheeling, so needs no further commands.
  motorDriveVolts = 0;

  lastCommandMillis = millis();

  velocityLimitCount = 0;
  positionLimitCount = 0;

  float p = disk.getDisplacement();

  if ( abs(p) <= positionEpsilon) { //disk is zero'd enough

	requestZeroPosition = false;

	controller.setLimits(positionPlantMin, positionPlantMax);

	// zero the commanded value (which wipes the error history)
	positionChangeCommand = 0;
	controller.setCommand(positionChangeCommand);

	report_integer = reportPosition;

	state = STATE_POSITION_READY;
	
	if (debug) Serial.println("Disk zeroed");
	
  } else {

	requestZeroPosition = true;

	if (debug) {
	  requestSerial();
	  Serial.print("Disk not zeroed, p = ");
	  Serial.println(p);
	  releaseSerial();
	}
	
    if (doReport) { //flag set in interrupt routine
	report();
	doReport = false; //clear flag so can run again later
	}
	
  }

}

void statePositionReady(void) {

  state = STATE_POSITION_READY;

  // this state just reports the current position
  
  // the PID coefficients can be updated at will
  // before starting the run.

  // do NOT update the PID controller in this state
  // so we keep zero'd history until we go to DURING
  
  if (doReport) { //flag set in interrupt routine
	report();
	doReport = false; //clear flag so can run again later
  }
}

void statePositionDuring(void) {

  state = STATE_POSITION_DURING; 

  float v, p, y, yp;
  float c, error, errp;
  
  if (doPID) {

    c = controller.getCommand();
	p = disk.getDisplacement();
    v = disk.getVelocity();


	if ( p < positionPlantMin || p > positionPlantMax ) {
	  state = STATE_POSITION_AFTER;
	} 
	
	y = controller.update(p);

	yp = driverPosition.drive(y,v);
	
	motor.drive(yp);
	motorDriveVolts = yp * motorMaxVolts;
  }

  if (doReport) { //flag set in interrupt routine
    //report();
	if (debug) {
	  Serial.print("c=");
	  Serial.print(positionToExternalUnits(controller.getCommand()));
	  Serial.print(", enc=");
	  Serial.print(count);
	  Serial.print(", p=");
	  Serial.print(positionToExternalUnits(p));	  
	  Serial.print(", v=");
	  Serial.print(velocityToExternalUnits(v));
	  Serial.print(", err=");
	  Serial.print(positionToExternalUnits(c-p));
	  Serial.print(", y=");	  
	  Serial.print(positionToExternalUnits(y));
	  Serial.print(", e0=");	  
	  Serial.print(positionToExternalUnits(controller.getError()));	  
	  Serial.print(", *yp=");
	  Serial.println(yp);
	  
	} else  {
	  report();
	}
	doReport = false; //clear flag so can run again later
  }

  // Disk can go unstable, and oscillate over a large amplitude - prevent!
  if (p > positionLimitMax || p < positionLimitMin ) {
	positionLimitCount++;
  } else {
	positionLimitCount = 0;
  }
  
  if (positionLimitCount > positionLimitCountThreshold)  {
	state = STATE_POSITION_AFTER;
	Serial.println("{\"error\":\"position limit exceeded\"}");
	Serial.println("{​\"error\":\"limit\",\"type\":\"position\",\"state\":\"stopping\"}");
  }


  if (abs(disk.getVelocity()) > velocityLimit) {
	velocityLimitCount++;
  } else {
	velocityLimitCount = 0;
  }

  if (velocityLimitCount > velocityLimitCountThreshold) {
	state = STATE_MOTOR_AFTER;
	Serial.println("{\"error\":\"velocity limit exceeded\"}");
	Serial.println("{​\"error\":\"limit\",\"type\":\"velocity\",\"state\":\"stopping\"}");
  }
  
  // Disk can sometimes oscillate, so shutdown on timeout.
  if (millis() >= lastCommandMillis + shutdownTimeMillis) {
	Serial.println("{\"info\":\"maximum run time exceeded\"}");
	Serial.println("{​\"error\":\"limit\",\"type\":\"time\",\"state\":\"stopping\"}");
    state = STATE_POSITION_AFTER;
  }

}


void statePositionChangeCommand(void) {

  state = STATE_POSITION_DURING; //start the PID controller next, by going to DURING state

  lastCommandMillis = millis();
  
  if(positionChangeCommand >= positionCommandMin && positionChangeCommand <= positionCommandMax) {

    controller.setCommand(positionChangeCommand);
	Serial.print("{\"info\":\"new position command\",\"c\":\"");
	Serial.print(positionToExternalUnits(controller.getCommand()));
	Serial.println("\"}"); 
  } else {
    Serial.println("{\"error\":\"cannot command position outside range\"}");
	Serial.println("{​\"error\":\"command\",\"type\":\"limit\",\"state\":\"position\"}");
  }
}

void statePositionChangeParameters(void) {

  state = STATE_POSITION_READY; // we can only set params in the ready state, so return there

  lastCommandMillis = millis();

  bool adjustKp = isNewKp;
  
  changePIDCoefficients();

  // scale Kp during position mode - only rescale if a new Kp has been set
  // or else we'll change a number that should not change
  // must scale after calling changePIDCoeffficients so that the
  // unscaled value is reported
  
  if (adjustKp) {
	controller.setKp(controller.getKp() * positionPIDScaleFactor);
  }

}


void statePositionAfter(void) {

  state = STATE_STOPPING_BEFORE;

  // return to non-scaled Kp
  controller.setKp(controller.getKp() / positionPIDScaleFactor);

  // TODO set motor drive to zero
  
  if (debug) Serial.println("statePositionAfter");
		
}


//*****************************************************************************

//STATE MACHINE RUN FUNCTION
void SMRun(void)
{
  if (state < numStates)
  {
    state = readSerialJSON(state);      // check for incoming commands received
    (*StateMachine[state].func)();        //reads the current state and then runs the associated function

  }
  else {
    Serial.println("Exception in State Machine");
  }

}


//This function is run on a timer interrupt defined by PIDInterval/timer_interrupt_freq.
void TimerInterrupt(void) {
  lastCount = count;
  count = pulse;
  updated = true;
}

void counterA(void) {

  bool A = (PORT->Group[g_APinDescription[encoderPinA].ulPort].IN.reg & (1ul << g_APinDescription[encoderPinA].ulPin)) != 0 ;
  bool B = (PORT->Group[g_APinDescription[encoderPinB].ulPort].IN.reg & (1ul << g_APinDescription[encoderPinB].ulPin)) != 0 ;

  pulse += (A==B ? +1 : - 1);
}

void counterB(void) {

  bool A = (PORT->Group[g_APinDescription[encoderPinA].ulPort].IN.reg & (1ul << g_APinDescription[encoderPinA].ulPin)) != 0 ;
  bool B = (PORT->Group[g_APinDescription[encoderPinB].ulPort].IN.reg & (1ul << g_APinDescription[encoderPinB].ulPin)) != 0 ;

  pulse += (A!=B ? +1 : - 1);

}
//===================================================================================
//====================== SETUP AND LOOP =============================================
//===================================================================================

void setup() {
  if (permitOverspeed) {
	velocityLimit *=100;
  }

  attachInterrupt(digitalPinToInterrupt(encoderPinA), counterA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), counterB, CHANGE);
  
  driverMotor.threshold = 0.0; //don't use second curve
  driverMotor.primaryOffsetPos = motorPrimaryOffsetPos; 
  driverMotor.primaryOffsetNeg = motorPrimaryOffsetNeg; 
  driverMotor.primaryOffsetThreshold = 0; //rps
  
  driverPosition.threshold = 0.0; //don't use second curve
  driverPosition.primaryOffsetPos = positionPrimaryOffsetPos; 
  driverPosition.primaryOffsetNeg = positionPrimaryOffsetNeg; 
  driverPosition.primaryOffsetThreshold = 0; //rps
  
  driverVelocity.threshold = 0.0; //don't use second curve
  driverVelocity.primaryOffsetPos = velocityPrimaryOffsetPos;
  driverVelocity.primaryOffsetNeg = velocityPrimaryOffsetPos;
  driverVelocity.primaryOffsetThreshold = 0;
  
  lastCommandMillis = millis();

  Serial.setTimeout(50);
  Serial.begin(57600);
  initialiseDisk = true;
  requestZeroPosition = false;
  startTimer(timer_interrupt_freq);   //setup and start the timer interrupt functions for PID calculations

  while (! Serial);
}

void loop() {

    if (updated) {
	
	updated = false;
	
	if (requestZeroPosition) {
	 offset = count;
	 initialiseDisk = true;
	}

	// must ensure initialisation and first step
	// are done in separate time steps to avoid
	// polluting the velocity calculation with 
	// a small/zero time step
	if (initialiseDisk) {
	  disk.initialise(count-offset); 
	  initialiseDisk = false;
	} else {	
	  disk.sample(count-offset);
	}
	
	//tell statemachine to do PID, report if needed
	doPID = true; 
	doReport = true;

  }
  
  // update state machine (which will also run tasks flagged by interrupts)
  SMRun();

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


void newShutdownTimer(float time) {

  if (time == 0) return;

  time *= 1000; //convert from seconds to milliseconds
  
  if ( time <= longestShutdownTimeMillis) {
    shutdownTimeMillis = 1000;
  }
}

bool inRange(float val, float min, float max) {
  return (val >= min && val <= max);
}

void changePIDCoefficients(void) {

  if (isNewKp) {
    if (inRange(newKp, KpMin, KpMax)) {
      controller.setKp(newKp);
    }
    isNewKp = false;
  }

  if (isNewKi) {
    if (inRange(newKi, KiMin, KiMax)) {
      controller.setKi(newKi);
    }
    isNewKi = false;
  }
  if (isNewKd) {
    if (inRange(newKd, KdMin, KdMax)) {
      controller.setKd(newKd);
    }
    isNewKd = false;
  }

  if (isNewTs) {
    if (inRange(newTs, TsMin, TsMax)) {

	  Ts = newTs; 
	  controller.setTs(Ts);
	  
      setTimerFrequencyHz(hertzFromSeconds(Ts)); 
	  
    }
    isNewTs = false;
  }

  Serial.print("{\"info\":\"new PID parameters\"");
  Serial.print(",\"Kp\":");
  Serial.print(controller.getKp());
  Serial.print(",\"Ki\":");
  Serial.print(controller.getKi());
  Serial.print(",\"Kd\":");
  Serial.print(controller.getKd());  
  Serial.print(",\"Ts\":");
  Serial.print(controller.getTs());
  Serial.print(",\"N\":");
  Serial.print(controller.getN());
  Serial.println("}");
  
  // pid controller automatically resets its history
  // when parameters are changed

}

/*  Unit conversion functions
 *
 *  Our internal units are normalised
 *  to unity for convenience
 *  (e.g. 1.0 is a complete revolution for -0.5 to +0.5)
 *
 *  The external world requires convential units (obvs).
 *  The choice of convention is set by existing course materials
 *
 *  quantity | external     | internal
 *  ---------|--------------|--------- 
 *  position |  rad         | rad/TWO_PI
 *  velocity |  rad/sec     | rad/TWO_PI/sec
 *  time     |  millis      |  (various)
 *  interval |  millis (dt) |  seconds (Ts)
 *
 *  TWO_PI is defined in arduino.h
 *  
 *  report() is responsible for converting outgoing units
 *  readSerialJSON() is responseible for converting incoming units
 *
 *  For example:
 *    a position commanded as +PI/4 rad is treated internally 
 *    as +0.125 (1/8th of revolution from zero).
 *
 *    a velocity commanded as +20*PI rad/sec (~62.83) is treated internally 
 *    as +10.0 (10 revolutions per second)
 *    For appreciation of the velocity involved, note that
 *    1 revolution per second is ~6.28 rad/sec
 *    1 revolution per second is 60 rpm
 *    1 rad/sec is 9.55rpm ie. approx factor of 10 if doing mental estimates
 */


float positionToExternalUnits(float p) {
  return p * TWO_PI;
}

float positionFromExternalUnits(float p) {
  return p / TWO_PI; 
}


float velocityToExternalUnits(float v) {
  return v * TWO_PI;
}

float velocityFromExternalUnits(float v) {
  return v / TWO_PI;
}

float hertzFromSeconds(float t) {
  return 1.0f / t;
}


float hertzFromMillis(float t) {
  return 1000.0f / t;
}

float secondsFromMillis(float t) {
  return t / 1000.0f;
}

float secondsFromMicros(float t) {
  return t / 1000000.0f;
}



//===================================================================================
//======================  READ AND PARSE JSON COMMMANDS =============================
//
//  This function can and does change the state of the state machine
//
//===================================================================================

StateType readSerialJSON(StateType state) {

  if(Serial.available() > 0) {

    //TODO move to state logic
      //on any command sent, reset the start time for hardware switch off

    Serial.readBytesUntil(10, command, COMMAND_SIZE);
    deserializeJson(doc, command);

    const char* set = doc["set"];

	if(strcmp(set, "api")==0) {
	  apiVersion = doc["to"];

	} else if(strcmp(set, "volts")==0) {
	  if(state == STATE_MOTOR_DURING) {
        state = STATE_MOTOR_CHANGE_COMMAND;
        motorChangeCommand = doc["to"];
      } else {
        Serial.println("{\"error\":\"in wrong state to set volts\"}");
      }

	} else if(strcmp(set, "position")==0) {
      if(state == STATE_POSITION_READY || state == STATE_POSITION_DURING) {
        state = STATE_POSITION_CHANGE_COMMAND;
        positionChangeCommand = positionFromExternalUnits(doc["to"]);
      } else {
        Serial.println("{\"error\":\"in wrong state to set position\"}");
      }

	} else if(strcmp(set, "velocity")==0) {
      if(state == STATE_VELOCITY_DURING) {
        state = STATE_VELOCITY_CHANGE_COMMAND;
        velocityChangeCommand = velocityFromExternalUnits(doc["to"]);
		if (debug) {
		  Serial.print("velocityChangeCommand=");
		  Serial.println(velocityChangeCommand);
		}
      } else {
        Serial.println("{\"error\":\"in wrong state to set velocity\"}");
      }

	} else if(strcmp(set, "mode")==0) {

      const char* new_mode = doc["to"];

      if(state == STATE_STOPPED) {
        if(strcmp(new_mode, "velocity") == 0) {
          state = STATE_VELOCITY_BEFORE;
        }
        else if(strcmp(new_mode, "motor") == 0) {
          state = STATE_MOTOR_BEFORE;
        }
        else if(strcmp(new_mode, "position") == 0) {
          state = STATE_POSITION_BEFORE;
        }
      } else {
        if(strcmp(new_mode, "stop") == 0) {
          state = STATE_STOPPING_BEFORE;
        }
        if(strcmp(new_mode, "wait") == 0) {
		  if ( state == STATE_POSITION_DURING ) {
          state = STATE_POSITION_WAITING;
		  }
        }		
      }

    } else if(strcmp(set, "parameters")==0 || strcmp(set, "p")==0 ) {

      if(!doc["kp"].isNull()) {
        newKp = doc["kp"];
        isNewKp = true;
      } else {
        isNewKp = false;
      }

      if(!doc["ki"].isNull()) {
        newKi = doc["ki"];
        isNewKi = true;
      } else {
        isNewKi = false;
      }

      if(!doc["kd"].isNull()) {
        newKd = doc["kd"];
        isNewKd = true;
      } else {
        isNewKd = false;
      }

      if(!doc["ts"].isNull()) {
        newTs = secondsFromMillis(doc["ts"]);
        isNewTs = true;
      } else {
        isNewTs = false;
      }
      if(!doc["n"].isNull()) {
        newN = secondsFromMillis(doc["n"]);
        isNewN = true;
      } else {
        isNewN = false;
      }	  

      if (isNewKp || isNewKi || isNewKd || isNewTs || isNewN) {
        if ( state == STATE_POSITION_READY) {
          state = STATE_POSITION_CHANGE_PARAMETERS;
        } else if (state == STATE_VELOCITY_DURING) {
          state = STATE_VELOCITY_CHANGE_PARAMETERS;
        } else {
          if (debug) Serial.println("{\"error\":\"in wrong state to set coefficients\"}");
        }

      }

	  if(!doc["vlpf"].isNull()) {
		float lpf = doc["vlpf"];
		disk.setLPF(lpf);
      }
	  

    } else if (strcmp(set, "drive")==0) {
  
      if(!doc["mon"].isNull()) {
		float mon = doc["mon"];
		if (abs(mon) <= 1.0) {
		  driverMotor.primaryOffsetNeg = mon;
		}
	  }
      if(!doc["mop"].isNull()) {
		float mop = doc["mop"];
		if (abs(mop) <= 1.0) {
		  driverMotor.primaryOffsetPos = mop;
		}
	  }
      if(!doc["pon"].isNull()) {
		float pon = doc["pon"];
		if (abs(pon) <= 1.0) {
		  driverPosition.primaryOffsetNeg = pon;
		}
	  }
      if(!doc["pop"].isNull()) {
		float pop = doc["pop"];
		if (abs(pop) <= 1.0) {
		  driverPosition.primaryOffsetPos = pop;
		}
	  }	  
      if(!doc["von"].isNull()) {
		float von = doc["von"];
		if (abs(von) <= 1.0) {
		  driverVelocity.primaryOffsetNeg = von;
		}
	  }
      if(!doc["vop"].isNull()) {
		float vop = doc["vop"];
		if (abs(vop) <= 1.0) {
		  driverVelocity.primaryOffsetPos = vop;
		}
	  }
	  
	}  else if (strcmp(set, "show")==0) {

      const char* new_show = doc["to"];

      if (strcmp(new_show, "long")==0) {
        show_mode = SHOW_LONG;
      }
      else if (strcmp(new_show, "short")==0) {
        show_mode = SHOW_SHORT;
      }
      else if (strcmp(new_show, "none")==0) {
        show_mode = SHOW_NONE;
      }
      else if (strcmp(new_show, "fc")==0) {
        show_mode = SHOW_FC;
      }
    }
    else if(strcmp(set, "timer") == 0) {
      newShutdownTimer(doc["to"]);
    }
  
  const char* get = doc["get"];

  if (strcmp(get, "api")==0) {

	Serial.println("\"info\",\"api\",\"version\":2.0,\"name\":\"spinner\"}");

  } else if (strcmp(get, "pid")==0){
	//return pid parameters
	requestSerial();
	Serial.print("{\"info\",\"pid\", \"kp\":");
	Serial.print(controller.getKp());
	Serial.print(",\"ki\":");
	Serial.print(controller.getKi());	  
	Serial.print(",\"kd\":");
	Serial.print(controller.getKd());
	Serial.print(",\"ts\":");
	Serial.print(controller.getTs());
	Serial.print(",\"n\":");
	Serial.print(controller.getN());
	Serial.println("}");
	releaseSerial();
 
  } else if (strcmp(get, "drive")==0) {

	requestSerial();
	Serial.print("{\"info\",\"drive\", \"mon\":");
	Serial.print(driverMotor.primaryOffsetNeg);
	Serial.print(",\"mop\":");
	Serial.print(driverMotor.primaryOffsetPos);
	Serial.print(", \"pon\":");
	Serial.print(driverPosition.primaryOffsetNeg);
	Serial.print(",\"pop\":");
	Serial.print(driverPosition.primaryOffsetPos);	  
	Serial.print(",\"von\":");
	Serial.print(driverVelocity.primaryOffsetNeg);
	Serial.print(",\"vop\":");
	Serial.print(driverVelocity.primaryOffsetPos);
	Serial.println("}");
	releaseSerial();

  }else if (strcmp(get, "limits")==0) {
	
	requestSerial();
	Serial.print("{\"info\",\"limits\", \"mn\":");
    Serial.print(motorMinCommand);
	Serial.print(",\"mp\":");
	Serial.print(motorMaxCommand);
	Serial.print(",\"pn\":");	
    Serial.print(positionCommandMin);
	Serial.print(",\"pp\":");
	Serial.print(positionCommandMax);
	Serial.print(",\"vn\":");	
    Serial.print(velocityCommandMin);
	Serial.print(",\"vp\":");
	Serial.print(velocityCommandMax);	
	Serial.println("}");
	releaseSerial();
	
  }  else if (strcmp(get, "units")==0) {
	requestSerial();
	Serial.println("{\"info\":\"units\",\"p\":\"rad\",\"v\":\"rad/s\",\"t\":\"s\"}");
	releaseSerial();
  }
  else if (strcmp(get, "uptime")==0) {
	//return seconds

  }
  }
  
  return state;     //return whatever state it changed to or maintain the state.
}

void requestSerial(void){
  while(writing); //wait for port to free up
  writing = true;
}

void releaseSerial(void){
  writing = false;
}

//===================================================================================
//======================  REPORT ====================================================
//
//===================================================================================

void report(void)
{
  // Don't detach interrupts because then we lose counts.
  // Let the outside world handle corrupted messages safely

  if (show_mode == SHOW_NONE) {
    return; // do nothing
  }

  requestSerial();

  if (apiVersion == 0) {

	reportCount ++;

	if ( reportCount  == 4 ) { //only send data every 20ms

	  reportCount = 0;
	  
	  Serial.print("{\"d\":");
	  Serial.print(positionToExternalUnits(disk.getDisplacement()));
	  Serial.print(",\"v\":");
	  Serial.print(velocityToExternalUnits(disk.getVelocity()));
	  Serial.print(",\"t\":");
	  Serial.print(millis());
	  Serial.print(",\"y\":");
	  Serial.print(motorDriveVolts);
	  
	  if (state == STATE_POSITION_DURING) {
		
		Serial.print(",\"c\":");
		Serial.print(positionToExternalUnits(controller.getCommand()));
		Serial.print(",\"p_sig\":");
		Serial.print(positionToExternalUnits(controller.getError()));
		Serial.print(",\"i_sig\":0,\"d_sig\":0");
		Serial.print(",\"e\":");
		Serial.print(positionToExternalUnits(controller.getError()));		
		Serial.print(",\"m\":\"p\"");
		Serial.print(",\"y\":");
		Serial.print(motorDriveVolts);
					 
		
		
	  } else if (state == STATE_VELOCITY_DURING) {
		
		Serial.print(",\"c\":");
		Serial.print(velocityToExternalUnits(controller.getCommand()));
		Serial.print(",\"p_sig\":");
		Serial.print(velocityToExternalUnits(controller.getError()));
		Serial.print(",\"i_sig\":0,\"d_sig\":0");
		Serial.print(",\"e\":");
		Serial.print(velocityToExternalUnits(controller.getError()));		
		Serial.print(",\"m\":\"v\"");
		Serial.print(",\"y\":");
		Serial.print(motorDriveVolts);
	  } else if (state == STATE_MOTOR_DURING) {

		Serial.print(",\"m\":\"m\"");
		
	  } else {
		
		Serial.print(",\"m\":\"s\"");
	  }
	  
	  
	  Serial.println("}");
	}

  }
  
  releaseSerial();
  doReport = false;
}

 

//===================================================================================
//======================TIMER INTERRUPT FUNCTIONS====================================
//      FROM https://github.com/nebs/arduino-zero-timer-demo/
//===================================================================================

void setTimerFrequencyHz(int frequencyHz) {
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

  setTimerFrequencyHz(frequencyHz);

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
