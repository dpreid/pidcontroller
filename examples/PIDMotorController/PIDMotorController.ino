/*
 * PIDMotorController
 * David Reid
 * 16/09/20
 * 
 * Based upon Tim Drysdale's penduino code.
 * https://github.com/timdrysdale/penduino
 * 
 * Timer interrupt functions from https://github.com/nebs/arduino-zero-timer-demo/
 */


#include <QueueArray.h>

#include <MotorController.h>
#include "ArduinoJson-v6.9.1.h"


// Pins on Motor Driver board.
#define AIN1 9
#define AIN2 10
#define PWMA 6
#define STBY 8

#define encoderPinA 2     //these pins all have interrupts on them.
#define encoderPinB 3
#define indexPin 11

//LED output pins
#define ledPositive 14
#define ledIndex 15
#define ledNegative 16
#define ledPowerOn 17

bool debug = false;
unsigned long report_interval = 10;
bool encoderPlain = true;

//for both PID modes and error calculations
volatile int encoderPos = 0;
volatile int encoderPosLast = 0;
volatile float encoderAngVel = 0;
//volatile float encoderAngVelLast = 0;
volatile int encoder_direction = 1;     //+1 CCW, -1 CW.
volatile int encoder_direction_last = 1;

unsigned long current_time_index = 0;
unsigned long previous_time_index = 0;

//other user set values
int set_position = 0;     //the position the user has set
int set_speed = 0;        //user set position for PID_Speed mode and DC_Motor mode
volatile float Kp = 0.5;               //PID parameters
volatile float Ki = 0.5;
volatile float Kd = 0;
float maxKp = 10;
float maxKi = 10;
float maxKd = 10;

//pid calculated values
volatile int error = 0;
volatile int previous_error = 0;
volatile float error_speed = 0;
volatile float previous_error_speed = 0;
float previous_errors[10];
volatile float proportional_term = 0;
volatile float integral_term = 0;
volatile float error_sum = 0;
volatile float derivative_term = 0;
volatile float PID_signal = 0;

QueueArray <float> errors_queue;    //queue of previous errors 
QueueArray <float> errors_speed_queue;
int numErrors = 10;

boolean A_set = false;
boolean B_set = false;

//JSON serialization
#define COMMAND_SIZE 64
StaticJsonDocument<COMMAND_SIZE> doc;
char command[COMMAND_SIZE];

// If motor spins in the opposite direction then you can change this to -1.
const int offset = 1;

bool led_index_on = false;

Motor motor = Motor(AIN1, AIN2, PWMA, offset, STBY);

int speed_limit = 100;
int position_limit = 1000;    //the number of encoderPos intervals in half a rotation (encoder rotates from -1000 to 1000).

int pid_interval = 1;       //ms, for a simple timer interval to run calculation of PID parameters. Do I need to include a true timer interrupt?

int sameNeeded = 100;        //number of loops with the same encoder position to assume that motor is stopped.
int stopped_count = 0;
int stopped_last_pos = 0;

bool doInterruptAB = false;
bool doInterruptIndex = false;

#define CPU_HZ 48000000
#define TIMER_PRESCALER_DIV 1024
float timer_interrupt_freq = 1000/pid_interval;   //200Hz once every 5ms
/**
 * Defines the valid states for the state machine
 * 
 */
typedef enum
{
  STATE_CALIBRATE,      //state for resetting the encoder 0 position
  STATE_AWAITING_STOP,  //checking if the motor has stopped
  STATE_STOPPED,        //no drive to motor
  STATE_PID_SPEED_MODE, //pid controller mode - 1st order, speed functions
  STATE_PID_POSITION_MODE,  //pid controller - 2nd order, position functions
  STATE_DC_MOTOR_MODE,    //regular dc motor functions with no PID controller
  STATE_AWAITING_NEW_COMMAND,
  STATE_CHANGING_INERTIA, //mode for changing position of the governor.
} StateType;

//state Machine function prototypes
//these are the functions that run whilst in each respective state.
void Sm_State_Start_Calibration(void);
void Sm_State_Awaiting_Stop(void);
void Sm_State_Stopped(void);
void Sm_State_PID_Speed(void);
void Sm_State_PID_Position(void);
void Sm_State_DC_Motor(void);
void Sm_State_Awaiting_New_Command(void);
void Sm_State_Change_Inertia(void);

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
  {STATE_CALIBRATE, Sm_State_Start_Calibration},
  {STATE_AWAITING_STOP, Sm_State_Awaiting_Stop},
  {STATE_STOPPED, Sm_State_Stopped},
  {STATE_PID_SPEED_MODE, Sm_State_PID_Speed},
  {STATE_PID_POSITION_MODE, Sm_State_PID_Position},
  {STATE_DC_MOTOR_MODE, Sm_State_DC_Motor},
  {STATE_AWAITING_NEW_COMMAND, Sm_State_Awaiting_New_Command},
  {STATE_CHANGING_INERTIA, Sm_State_Change_Inertia}
};
 
int NUM_STATES = 8;

/**
 * Stores the current state of the state machine
 */
 
StateType SmState = STATE_STOPPED;    //START IN THE STOPPED STATE

//DEFINE STATE MACHINE FUNCTIONS================================================================


void Sm_State_Stopped(void){
  motor.brake();

  doInterruptAB = false;
  doInterruptIndex = false;
  
  //report_encoder();
  SmState = STATE_STOPPED;
}

void Sm_State_Awaiting_Stop(void){
  bool moving = true;
  int lastPos = 0;
  int thisPos = 0;
  //int sameNeeded = 10;
  int sameCount = sameNeeded;   //sameNeeded has become a global variable
  while (moving)
  {
    lastPos = thisPos;
    thisPos = encoderPos;
    if (thisPos == lastPos){
      sameCount = sameCount - 1;
    }
    else{
      sameCount = sameNeeded;
    }
      if (encoderPlain){
        Serial.println(encoderPos);
      }
      else{
        Serial.print("{\"cal\":true,\"enc\":");
        Serial.print(encoderPos);
        Serial.print(",\"sameCount\":");
        Serial.print(sameCount);
        Serial.print(",\"time\":");
        Serial.print(millis());      
        Serial.println("}");
      }
    if (sameCount <= 0){
      moving = false;
    }
  }

  SmState = STATE_STOPPED;    //transition to the stopped state.
}

void Sm_State_Awaiting_New_Command(void){
  motor.brake();
  //report_encoder();
  
  SmState = STATE_AWAITING_NEW_COMMAND;
}

void Sm_State_PID_Position(void){
    doInterruptAB = true;
    doInterruptIndex = false;
    
  //calculate the PID parameters but limit to specific frequency
//  if(millis() % pid_interval == 0){
//      calculatePositionPID();
//      //Check_Stopped();
//    }

    

//limit the rotation speed for now!

   if(PID_signal >=-speed_limit && PID_signal <= speed_limit){  
      motor.drive(PID_signal); 
   } else if(PID_signal > speed_limit){
      motor.drive(speed_limit);
   } else{
      motor.drive(-speed_limit);
   }

   setRotationLEDs(PID_signal);
  
  //Serial.println(error);
  report_encoder();
  SmState = STATE_PID_POSITION_MODE;

  
}

void Sm_State_PID_Speed(void){

    doInterruptAB = false;
    doInterruptIndex = true;
    
//limit the rotation speed for now!
   if(PID_signal >=-speed_limit && PID_signal <= speed_limit){  
      motor.drive(PID_signal); 
   } else if(PID_signal > speed_limit){
      motor.drive(speed_limit);
   } else{
      motor.drive(-speed_limit);
   }

   setRotationLEDs(PID_signal);
  Serial.print("error value = ");
  Serial.println(error_speed);
  Serial.print("encoder speed = ");
  Serial.println(encoderAngVel);
  Serial.print("PID_signal = ");
  Serial.println(PID_signal);
  //report_encoder_speed();
  SmState = STATE_PID_SPEED_MODE;
}

void Sm_State_Start_Calibration(void){
  Serial.println("Entered calibration mode");
  
  
}

void Sm_State_DC_Motor(void){
  motor.drive(set_speed);

  doInterruptAB = false;
  doInterruptIndex = true;
  
  //set led pins
  setRotationLEDs(set_speed);

  Serial.println(encoder_direction);
  //report_encoder_speed();
  SmState = STATE_DC_MOTOR_MODE;
}

void Sm_State_Change_Inertia(void){
  Serial.println("Entered change inertia mode");
}

//STATE MACHINE RUN FUNCTION
void Sm_Run(void)
{
  if (SmState < NUM_STATES)
  {
    SmState = readSerialJSON(SmState);      
    (*StateMachine[SmState].func)();        //reads the current state and then runs the associated function
    
  }
  else{
    Serial.println("Exception in State Machine");
  }
  
}

//This function is run on a timer interrupt defined by timer_interrupt_freq.
void TimerInterrupt(void){
  if(SmState == STATE_PID_POSITION_MODE){
     calculatePositionPID();
  } else if(SmState == STATE_PID_SPEED_MODE){
    calculateSpeedPID();
  } 
}

// SETUP AND LOOP==================================================================================
void setup() {
  //setup encoder pins
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  pinMode(indexPin, INPUT);
  digitalWrite(encoderPinA, HIGH);  // turn on pull-up resistor
  digitalWrite(encoderPinB, HIGH);  // turn on pull-up resistor
  digitalWrite(indexPin, HIGH);

  //setup led output pins
  pinMode(ledPositive, OUTPUT);
  pinMode(ledIndex, OUTPUT);
  pinMode(ledNegative, OUTPUT);
  pinMode(ledPowerOn, OUTPUT);
  
  digitalWrite(ledPowerOn, HIGH);   //just to show that arduino is on.
  
  //TESTING SWITCHING INTERRUPTS ON/OFF FOR DIFFERENT MODES
  
  // encoder pin on interrupt (pin 2)
  attachInterrupt(digitalPinToInterrupt(encoderPinA), doEncoderA, CHANGE);
  // encoder pin on interrupt (pin 3)
  attachInterrupt(digitalPinToInterrupt(encoderPinB), doEncoderB, CHANGE);
// encoder pin on interrupt (pin 11)
  attachInterrupt(digitalPinToInterrupt(indexPin), doIndexPin, RISING);

  current_time_index = millis();   
  previous_time_index = millis();

  Serial.setTimeout(50);
  Serial.begin(57600);

  startTimer(timer_interrupt_freq);

   while (! Serial);

   
}

void loop() {
  
  Sm_Run();  
}

//POSSIBLE COMMANDS:
//set_position - only runs in STATE_PID_POSITION mode, takes in a parameter between -1000 and 1000 for the encoder position.
//set_speed - only runs in STATE_PID_SPEED_MODE, takes a parameter between -255, 255. Can be limited by speed_limit variable.
//change_mode - changes between PID_SPEED, PID_POSITION, DC_MOTOR, CHANGE_INERTIA and AWAITING_STOP modes.
// set_parameters - to set the PID parameters Kp, Ki and Kd as well as any other necessary parameters.

StateType readSerialJSON(StateType SmState){
  if(Serial.available() > 0){
  
    Serial.readBytesUntil(10, command, COMMAND_SIZE);
    deserializeJson(doc, command);

    const char* cmd = doc["cmd"];
    
    if(strcmp(cmd, "set_position")==0){
      if(SmState == STATE_PID_POSITION_MODE || SmState == STATE_AWAITING_NEW_COMMAND){
        SmState = STATE_PID_POSITION_MODE;    //set state
        stopped_count = 0;                    //and reset count for checking stopped.
        int new_position = doc["param"];
        if(new_position >= -position_limit && new_position <= position_limit){
          set_position = new_position;
          Serial.print("position set to: ");
          Serial.println(set_position);
        } else{
          Serial.print("position needs to be between -");
          Serial.print(position_limit);
          Serial.print(" and ");
          Serial.println(position_limit);
        }
      } else{
          Serial.println("In wrong state to set position");
      }
      
      
  } else if(strcmp(cmd, "set_speed")==0){
      if(SmState == STATE_PID_SPEED_MODE || SmState == STATE_DC_MOTOR_MODE){
        int new_speed = doc["param"];
        set_speed = new_speed;
      } else{
        Serial.println("In wrong state to set speed");
      }
      
    } 
    else if(strcmp(cmd, "set_mode")==0){
      
      const char* new_mode = doc["param"];
      if(strcmp(new_mode, "PID_SPEED_MODE") == 0){
        SmState = STATE_PID_SPEED_MODE;
      } 
      else if(strcmp(new_mode, "PID_POSITION_MODE") == 0){
        SmState = STATE_PID_POSITION_MODE;
      }
      else if(strcmp(new_mode, "DC_MOTOR_MODE") == 0){
        SmState = STATE_DC_MOTOR_MODE;
      }
      else if(strcmp(new_mode, "CHANGING_INERTIA_MODE") == 0){
        SmState = STATE_CHANGING_INERTIA;
      }
      else if(strcmp(new_mode, "STOP") == 0){
        SmState = STATE_STOPPED;          //NO AWAITING STOP CURRENTLY.
                          
      }
      
    } else if(strcmp(cmd, "set_parameters")==0){
      int new_Kp = doc["Kp"];
      int new_Ki = doc["Ki"];
      int new_Kd = doc["Kd"];

      if(new_Kp >= -maxKp && new_Kp <= maxKp){
        Kp = new_Kp;
      } else if(new_Kp > maxKp){
        Kp = maxKp;
      } else{
        Kp = -maxKp;
      }

      if(new_Ki >= -maxKi && new_Ki <= maxKi){
        Ki = new_Ki;
      } else if(new_Ki > maxKi){
        Ki = maxKi;
      } else{
        Ki = -maxKi;
      }

      if(new_Kd >= -maxKd && new_Kd <= maxKd){
        Kd = new_Kd;
      } else if(new_Kd > maxKd){
        Kd = maxKd;
      } else{
        Kd = -maxKd;
      }
    }



    
  }
      return SmState;     //return whatever state it changed to or maintain the state.
 } 





//outputs encoder position to serial bus.
void report_encoder(void)
{
  
 if (millis() % report_interval == 0){
      if (encoderPlain){
        Serial.println(encoderPos);
      }
      else{
        Serial.print("{\"enc\":");
        Serial.print(encoderPos);
        Serial.print(",\"time\":");
        Serial.print(millis());  
        Serial.println("}");
      }
    }
  
}

//outputs encoder speed to serial bus.
void report_encoder_speed(void){
   if (millis() % report_interval == 0){
      if (encoderPlain){
        Serial.println(encoderAngVel);
      }
      else{
        Serial.print("{\"enc_ang_vel\":");
        Serial.print(encoderAngVel);
        Serial.print(",\"time\":");
        Serial.print(millis());  
        Serial.println("}");
      }
    }
}

// Interrupt on A changing state
//CURRENTLY ALWAYS DOING ENCODER A INTERRUPT IN ORDER TO GET DIRECTION
void doEncoderA() {
  //Serial.println("checking A");
  // Test transition
  A_set = digitalRead(encoderPinA) == HIGH;
  // and adjust counter + if A leads B
  encoderPosLast = encoderPos;
  encoderPos += (A_set != B_set) ? +1 : -1;
  encoderWrap();
  
}

// Interrupt on B changing state
void doEncoderB() {
  //if(doInterruptAB){
  //Serial.println("checking B");
  // Test transition
  B_set = digitalRead(encoderPinB) == HIGH;
  // and adjust counter + if B follows A
  encoderPosLast = encoderPos;
  encoderPos += (A_set == B_set) ? +1 : -1;
  encoderWrap();

 
  //}
}

void doIndexPin(void){
  if(doInterruptIndex){
    //get direction of rotation
    encoder_direction_last = encoder_direction;
    if(encoderPos - encoderPosLast >= 0){
      encoder_direction = -1;
    } else{
      encoder_direction = 1;
    }
  
    previous_time_index = current_time_index;
    current_time_index = millis();
  
    encoderAngVel = encoder_direction * 60000.0/(current_time_index - previous_time_index);    //rpm

    led_index_on = !led_index_on;
    digitalWrite(ledIndex, led_index_on);
  }
  
}

void encoderWrap(void){
  if (encoderPos > position_limit) {
    encoderPos -= 2*position_limit;
    } else if (encoderPos < -position_limit) {
      encoderPos += 2*position_limit;
      }
}

void calculatePositionPID(){
  previous_error = error;
  int not_through_wrap_error = encoderPos - set_position;
  int mag = abs(not_through_wrap_error);
  int dir = not_through_wrap_error / mag;    //should be +1 or -1.
  
  int through_wrap_error = position_limit - abs(encoderPos) + position_limit - abs(set_position);

  if(abs(not_through_wrap_error) <= through_wrap_error){
    error = not_through_wrap_error;
  } else {
    error = -1*dir*through_wrap_error;
  }
  //error = encoderPos - set_position;
  
  //unsigned long delta_t = 2*(current_time - previous_time);
  int delta_t = pid_interval;
  
  proportional_term = error * Kp;
  derivative_term = (error - previous_error)*Kd / delta_t; 

    if(errors_queue.count() <= numErrors){
      errors_queue.push(error);
      error_sum += error;
    } else{
      float e = errors_queue.pop();
      error_sum -= e;
      errors_queue.push(error);
      error_sum += error;
    }
    integral_term = error_sum * Ki;

    PID_signal = proportional_term + integral_term + derivative_term;
}


void calculateSpeedPID(){
  previous_error_speed = error_speed;

  error_speed = set_speed - encoderAngVel;
  //error_speed = encoderAngVel - set_speed;
  
  int delta_t = pid_interval;
  
  proportional_term = error_speed * Kp;
  derivative_term = (error_speed - previous_error_speed)*Kd / delta_t; 

    if(errors_speed_queue.count() <= numErrors){
      errors_speed_queue.push(error_speed);
      error_sum += error_speed;
    } else{
      float e = errors_speed_queue.pop();
      error_sum -= e;
      errors_speed_queue.push(error_speed);
      error_sum += error_speed;
    }
    integral_term = error_sum * Ki;

    PID_signal = proportional_term + integral_term + derivative_term;
}


//This is similar to the Awaiting stop state, however, it cannot interrupt the motor functions as it is checking
//whether the motor has stopped, not waiting until it does.
void Check_Stopped(void){
  if(encoderPos == stopped_last_pos){
    Serial.print("Stopped count = ");
    Serial.println(stopped_count);
    stopped_count++;
  } else {
    stopped_count = 0;
  }

  
  if(stopped_count >= sameNeeded){
    Serial.println("Stopped");
    SmState = STATE_AWAITING_NEW_COMMAND;    //transition to the new state.
  }

  stopped_last_pos = encoderPos;
  
}

void setRotationLEDs(float value){
  if(value > 0){
    digitalWrite(ledPositive, HIGH);
    digitalWrite(ledNegative, LOW);
  } else if(value < 0){
    digitalWrite(ledPositive, LOW);
    digitalWrite(ledNegative, HIGH);
  } else{
    digitalWrite(ledPositive, LOW);
    digitalWrite(ledNegative, LOW);
  }
}

//==================================================================================
//======================TIMER INTERRUPT FUNCTIONS====================================
//FROM https://github.com/nebs/arduino-zero-timer-demo/
//==================================================================================



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
