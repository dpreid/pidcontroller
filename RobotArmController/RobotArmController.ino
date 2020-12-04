
/*
 * Remote Labs: Robot Arm Controller
 * David Reid
 * 02/12/20
 * 
 * Based upon Tim Drysdale's penduino code.
 * https://github.com/timdrysdale/penduino
 * 
 * Timer interrupt functions from https://github.com/nebs/arduino-zero-timer-demo/
 */
#include <Adafruit_NeoPixel.h>
#include <Servo.h>
#include <MotorControllerPmodHB3.h>
#include "ArduinoJson-v6.9.1.h"

//NeoPixel LED setup
#define NEOPIXEL_PIN 14
#define NUMPIXELS 8
Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
// Pins on Motor Driver board.
#define AIN1 5
#define PWMA 6

//define servo motor control
#define SERVO 18
Servo servo;
volatile float set_arm_extension = 90.0;   //the position of the servo motor for extending arm: angle (degrees) between 0 and 180.
 
#define encoderPinA 3     //these pins all have interrupts on them.
#define encoderPinB 2
#define indexPin 11

bool debug = false;
unsigned long report_interval = 10;   //ms
unsigned long previous_report_time = 0;
bool encoderPlain = false;

//for both PID modes and error calculations
volatile int encoderPos = 0;
volatile int encoderPosLast = 0;
volatile float encoderAngVel = 0;
volatile int encoder_direction = 1;     //+1 CCW, -1 CW.
volatile int encoder_direction_last = -1;
volatile int encoder_direction_index = 1;

unsigned long current_time_encoder = 0;
unsigned long previous_time_encoder = 0;
unsigned long current_time_index = 0;
unsigned long previous_time_index = 0;
unsigned long min_rotation_time = 6;      //any time difference smaller than 6ms won't be used to calculate angular velocity

//other user set values
float set_position = 0;     //the position the user has set
float set_speed = 0;        //user set position for PID_Speed mode and DC_Motor mode
float Kp = 1.0;               //PID parameters
float Ki = 0.0;
float Kd = 0.0;

//pid calculated values
volatile float error = 0;
volatile float previous_error = 0;
volatile float previous_previous_error = 0;

volatile float proportional_term = 0;
volatile float integral_term = 0;
volatile float error_sum = 0;
volatile float derivative_term = 0;
volatile float PID_signal = 0;
volatile float previous_PID_signal = 0;
volatile float previous_previous_PID_signal = 0;

boolean A_set = false;
boolean B_set = false;

//JSON serialization
#define COMMAND_SIZE 64  //originally 64
StaticJsonDocument<COMMAND_SIZE> doc;
char command[COMMAND_SIZE];

// If motor spins in the opposite direction then you can change this to -1.
const int offset = 1;

bool led_index_on = false;

MotorHB3 motor = MotorHB3(AIN1, PWMA, offset);

int position_max = 1000;    //the number of encoderPos intervals in half a rotation (encoder rotates from -1000 to 1000).
int position_limit = 200;   //the soft position limit either side of the zero point.

float zero_error = 10;
float zero_offset = -200;    //the encoderPos difference between index and digger arm 0 position

int pid_interval = 3;       //ms, for timer interrupt

int sameNeeded = 100;        //number of loops with the same encoder position to assume that motor is stopped.

bool doInterruptAB = false;
bool doInterruptIndex = false;

#define CPU_HZ 48000000
#define TIMER_PRESCALER_DIV 1024
float timer_interrupt_freq = 1000.0/pid_interval;   

/**
 * Defines the valid states for the state machine
 * 
 */
typedef enum
{
  STATE_ZERO,           //zeroes the index position
  STATE_OFFSET,          //zeroes the digger arm position
  STATE_AWAITING_STOP,  //checking if the motor has stopped
  STATE_STOPPED,        //no drive to motor
  STATE_PID_POSITION_MODE,  //pid controller - 2nd order, position functions
  STATE_CHANGE_ARM      //state for extending and retracting digger arm
} StateType;

//state Machine function prototypes
//these are the functions that run whilst in each respective state.
void Sm_State_Zero(void);
void Sm_State_Offset(void);
void Sm_State_Awaiting_Stop(void);
void Sm_State_Stopped(void);
void Sm_State_PID_Position(void);
void Sm_State_Change_Arm(void);

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
  {STATE_ZERO, Sm_State_Zero},
  {STATE_OFFSET, Sm_State_Offset},
  {STATE_AWAITING_STOP, Sm_State_Awaiting_Stop},
  {STATE_STOPPED, Sm_State_Stopped},
  {STATE_PID_POSITION_MODE, Sm_State_PID_Position},
  {STATE_CHANGE_ARM, Sm_State_Change_Arm},
};
 
int NUM_STATES = 6;

/**
 * Stores the current state of the state machine
 */
 
StateType SmState = STATE_STOPPED;    //START IN THE STOPPED STATE

//DEFINE STATE MACHINE FUNCTIONS================================================================

//TRANSITION: Stopped -> Stopped
void Sm_State_Stopped(void){  

  if(servo.attached()){
    servo.detach();
  }
  //doInterruptAB = false;
  doInterruptIndex = true;       
  set_position = encoderPos;   //reset the user set values so that when it re-enters a PID mode it doesn't start instantly
  set_speed = 0;
  encoderAngVel = 0;
 
  report_encoder();
  SmState = STATE_STOPPED;
}

//TRANSITION: AWAITING_STOP -> STOPPED
void Sm_State_Awaiting_Stop(void){
  motor.brake();
  
  bool moving = true;
  int lastPos = 0;
  int thisPos = 0;
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
        Serial.print("{\"awaiting_stop\":true,\"enc\":");
        Serial.print(encoderPos);
        Serial.print(",\"enc_ang_vel\":");
        Serial.print(encoderAngVel);
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

//TRANSITION: PID_POSITION -> PID_POSITION
void Sm_State_PID_Position(void){
    //doInterruptAB = true;
    doInterruptIndex = true;

    if(PID_signal > 255){
      motor.drive(255);
    } else if(PID_signal < -255){
      motor.drive(-255);
    } else{
      motor.drive(PID_signal);
    }
  
  report_encoder();
  SmState = STATE_PID_POSITION_MODE;

}

//TRANSITION: ZERO -> OFFSET
void Sm_State_Zero(void){
  
  //doInterruptAB = false;
  doInterruptIndex = true;

  bool index_state = led_index_on;
  float starting_signal = 50;
  while(index_state == led_index_on){
    motor.drive(-encoder_direction_index * starting_signal);
    starting_signal += 0.000001;
  }
  motor.brake();

  encoderPos = 0;

  delay(100);
  
  //report_encoder();
  if(encoderPos > zero_error || encoderPos < -zero_error){    //allowed calibration error
    SmState = STATE_ZERO;  
  } else{
    //index pin has been zeroed so...
    //transition to OFFSET STATE
    SmState = STATE_OFFSET;  
  }
}

//TRANSITION: OFFSET -> STOPPED
void Sm_State_Offset(void){
  //enter this state with the index pin having been zeroed and encoder position set to zero within a zero_error
  //doInterruptAB = false;
  doInterruptIndex = true;

  bool index_state = led_index_on;
  float starting_signal = 50;
  while(encoderPos != zero_offset){
    float diff = encoderPos - zero_offset;
    int drive_dir = diff/abs(diff);
    motor.drive(drive_dir * starting_signal);
    starting_signal += 0.000001;
  }
  motor.brake();

  //encoderPos = 0;

  delay(100);
  
  //report_encoder();
  if(encoderPos > (zero_offset + zero_error) || encoderPos < (zero_offset - zero_error)){    //allowed calibration error
    SmState = STATE_OFFSET;  
  } else{
    encoderPos = 0;         //this is the newly set 0 encoder position
    SmState = STATE_STOPPED;  
  }
}

//TRANSITION: CHANGE_ARM -> CHANGE_ARM
void Sm_State_Change_Arm(void){

  if(!servo.attached()){
    servo.attach(SERVO);
  }
  
  //if the servo set position has changed since the last call to write then call write
  if(servo.read() != set_arm_extension){
      servo.write(set_arm_extension);
  }

  //servo.detach();

  //SmState = STATE_STOPPED;
  SmState = STATE_CHANGE_ARM;
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

//This function is run on a timer interrupt defined by pid_interval/timer_interrupt_freq.
void TimerInterrupt(void){
  if(SmState == STATE_PID_POSITION_MODE){
     calculatePositionPID();
  } 
}

// SETUP AND LOOP==================================================================================
void setup() {
  //setup encoder pins
//  pinMode(encoderPinA, INPUT_PULLUP);   //pullup resistors not on board
//  pinMode(encoderPinB, INPUT_PULLUP);
//  pinMode(indexPin, INPUT_PULLUP);

  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  pinMode(indexPin, INPUT);

  //pinMode(SERVO, OUTPUT);
  //servo.attach(SERVO);

  pixels.begin(); // INITIALIZE NeoPixel
  
  attachEncoderInterrupts(); 

  current_time_index = millis();   
  previous_time_index = millis();

  previous_report_time = millis();

  Serial.setTimeout(50);
  Serial.begin(57600);

  startTimer(timer_interrupt_freq);   //setup and start the timer interrupt functions for PID calculations

   while (! Serial);

   
}

void loop() {
  
  Sm_Run();  
}


StateType readSerialJSON(StateType SmState){
  if(Serial.available() > 0){
  
    Serial.readBytesUntil(10, command, COMMAND_SIZE);
    deserializeJson(doc, command);
    
    const char* cmd = doc["cmd"];

    if(strcmp(cmd, "set_position")==0){
      if(SmState == STATE_PID_POSITION_MODE){
        float new_position = doc["param"];
        if(new_position >= -position_limit && new_position <= position_limit){
          
          resetPIDSignal();
          set_position = new_position;
          
        } else{
          Serial.println("Outside position range");
        }
      } else{
          Serial.println("In wrong state to set position");
      }
      
      
  } 
  else if(strcmp(cmd, "set_arm")==0){
    if(SmState == STATE_CHANGE_ARM){
      float new_position = doc["param"];
      if(new_position >= 0 && new_position <= 180){
        set_arm_extension = new_position;
      } else{
        Serial.println("Outside position range");
      }
    } else{
      Serial.println("In wrong state to set arm");
    }
  }
  else if(strcmp(cmd, "set_mode")==0){
      const char* new_mode = doc["param"];

      if(SmState == STATE_STOPPED){
        if(strcmp(new_mode, "PID_POSITION_MODE") == 0){
          SmState = STATE_PID_POSITION_MODE;
        }
        else if(strcmp(new_mode, "ZERO") == 0){
          SmState = STATE_ZERO;
        }
        else if(strcmp(new_mode, "CHANGE_ARM") == 0){
          SmState = STATE_CHANGE_ARM;
        }
       
      } else {
        
        if(strcmp(new_mode, "STOP") == 0){
          resetPIDSignal();
          SmState = STATE_AWAITING_STOP;      
        } 
        
      }
      
   
    } else if(strcmp(cmd, "set_parameters")==0){

      resetPIDSignal();

      if(!doc["Kp"].isNull()){
        Kp = doc["Kp"];
      }

      if(!doc["Ki"].isNull()){
        Ki = doc["Ki"];
      }

      if(!doc["Kd"].isNull()){
        Kd = doc["Kd"];
      }

      if(!doc["dt"].isNull()){
        pid_interval = doc["dt"];
        float timer_interrupt_freq = 1000/pid_interval;  
        setTimerFrequency(timer_interrupt_freq);        
      }
    } 
  }
      return SmState;     //return whatever state it changed to or maintain the state.
 } 


void resetPIDSignal(void){
  PID_signal = 0;
  previous_PID_signal = 0;
  previous_previous_PID_signal = 0;
  error = 0;
  previous_error = 0;
  previous_previous_error = 0;
}


//outputs encoder position and ang vel to serial bus.
void report_encoder(void)
{
  unsigned long current_time = millis();
 if (current_time >= previous_report_time + report_interval){
  
      if (encoderPlain){
        Serial.print("position = ");
        Serial.println(encoderPos);
        Serial.print("ang vel = ");
        Serial.println(encoderAngVel);
      }
      else{
        Serial.print("{\"enc\":");
        Serial.print(encoderPos);
        Serial.print(",\"enc_ang_vel\":");
        Serial.print(encoderAngVel);
        Serial.print(",\"time\":");
        Serial.print(current_time);  
        Serial.println("}");
      }

      previous_report_time = current_time;

    }
  
}

void attachEncoderInterrupts(void){
  attachInterrupt(digitalPinToInterrupt(encoderPinA), doEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), doEncoderB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(indexPin), doIndexPin, RISING);
}

// Interrupt on A changing state
//CURRENTLY ALWAYS DOING ENCODER INTERRUPTS IN ORDER TO GET DIRECTION
//Encoder A is used to calculate angular speed in rpm as well as new position
void doEncoderA() {
  A_set = digitalRead(encoderPinA) == HIGH;
  // and adjust counter + if A leads B
  encoderPosLast = encoderPos;
  encoderPos += (A_set != B_set) ? +1 : -1;


  if(encoderPos - encoderPosLast >= 0){
      encoder_direction = -1;
    } else{
      encoder_direction = 1;
    }

  //TESTING UPDATING THE ANGULAR VELOCITY ON ENCODER INTERRUPT AS WELL
//  if(A_set){
//    current_time_encoder = micros();
//    if(current_time_encoder > previous_time_encoder){
//      encoderAngVel = encoder_direction * 60000000.0/((current_time_encoder - previous_time_encoder)*500);    //rpm
//      previous_time_encoder = current_time_encoder;
//      } 
//
//  }

  encoderWrap();


}

// Interrupt on B changing state
void doEncoderB() {
  //if(doInterruptAB){
 
  B_set = digitalRead(encoderPinB) == HIGH;
  // and adjust counter + if B follows A
  encoderPosLast = encoderPos;
  encoderPos += (A_set == B_set) ? +1 : -1;
  encoderWrap();

  
  //}
}

//ENCODER DIRECTION IS ONLY NECESSARY FOR CALCULATING ANG VEL, SO ONLY NEEDS TO BE CORRECT WHEN
//INDEX PIN TRIGGERS. Error in direction on wrap is OK....?
void doIndexPin(void){
  if(doInterruptIndex){
    //get direction of rotation
    
    if(encoderPos - encoderPosLast >= 0){
      encoder_direction_index = -1;
    } else{
      encoder_direction_index = 1;
    }
  
//    previous_time_index = current_time_index;
//    current_time_index = millis();
//
//    if(current_time_index >= previous_time_index + min_rotation_time){
//      encoderAngVel = encoder_direction * 60000.0/(current_time_index - previous_time_index);    //rpm
//    }
    

    led_index_on = !led_index_on;
    setIndexLEDs(led_index_on);
    //digitalWrite(ledIndex, led_index_on);
    if(encoder_direction / encoder_direction_last < 0){
      setRotationLEDs(encoderAngVel);  
    }

    encoder_direction_last = encoder_direction;
  }
  
}

void encoderWrap(void){
  if (encoderPos > position_max) {
    encoderPos -= 2*position_max;
    } else if (encoderPos < -position_max) {
      encoderPos += 2*position_max;
      }
}

//DISCRETE TIME VERSION
void calculatePositionPID(void){
    previous_previous_error = previous_error;
    previous_error = error;
    
  float not_through_wrap_error = encoderPos - set_position;
  float mag = abs(not_through_wrap_error);
  int dir = not_through_wrap_error / mag;    //should be +1 or -1.
  
  float through_wrap_error = 2*position_max - abs(encoderPos) - abs(set_position);

  if(abs(not_through_wrap_error) <= through_wrap_error){
    error = not_through_wrap_error;
  } else {
    error = -1*dir*through_wrap_error;
  }
  //convert error to an angular error in deg
  error = error*180/position_max;
  
  
  float delta_t = pid_interval/1000.0;
  float Ti = Kp/Ki;
  float Td = Kd/Kp;
  
 float new_signal = Kp*(error - previous_error + delta_t*error/Ti +(Td/delta_t)*(error - 2*previous_error + previous_previous_error));

  PID_signal += new_signal;
    

}


void setStepperLEDs(float value){
  if(value > 0){
    pixels.setPixelColor(5, pixels.Color(0, 0, 150));
    pixels.setPixelColor(6, pixels.Color(0, 0, 150));
    pixels.setPixelColor(7, pixels.Color(0, 0, 150));
    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
    pixels.setPixelColor(1, pixels.Color(0, 0, 0));
    pixels.setPixelColor(2, pixels.Color(0, 0, 0));
    
  } else if (value < 0){
    pixels.setPixelColor(5, pixels.Color(0, 0, 0));
    pixels.setPixelColor(6, pixels.Color(0, 0, 0));
    pixels.setPixelColor(7, pixels.Color(0, 0, 0));
    pixels.setPixelColor(0, pixels.Color(0, 0, 150));
    pixels.setPixelColor(1, pixels.Color(0, 0, 150));
    pixels.setPixelColor(2, pixels.Color(0, 0, 150));
  } else{
    pixels.setPixelColor(5, pixels.Color(0, 0, 0));
    pixels.setPixelColor(6, pixels.Color(0, 0, 0));
    pixels.setPixelColor(7, pixels.Color(0, 0, 0));
    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
    pixels.setPixelColor(1, pixels.Color(0, 0, 0));
    pixels.setPixelColor(2, pixels.Color(0, 0, 0));
  }
    pixels.show();
}

void setRotationLEDs(float value){
  //pixels.clear();
  if(value > 0){
    pixels.setPixelColor(5, pixels.Color(0, 150, 0));
    pixels.setPixelColor(6, pixels.Color(0, 150, 0));
    pixels.setPixelColor(7, pixels.Color(0, 150, 0));
    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
    pixels.setPixelColor(1, pixels.Color(0, 0, 0));
    pixels.setPixelColor(2, pixels.Color(0, 0, 0));
    
  } else if (value < 0){
    pixels.setPixelColor(5, pixels.Color(0, 0, 0));
    pixels.setPixelColor(6, pixels.Color(0, 0, 0));
    pixels.setPixelColor(7, pixels.Color(0, 0, 0));
    pixels.setPixelColor(0, pixels.Color(0, 150, 0));
    pixels.setPixelColor(1, pixels.Color(0, 150, 0));
    pixels.setPixelColor(2, pixels.Color(0, 150, 0));
  } else{
    pixels.setPixelColor(5, pixels.Color(0, 0, 0));
    pixels.setPixelColor(6, pixels.Color(0, 0, 0));
    pixels.setPixelColor(7, pixels.Color(0, 0, 0));
    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
    pixels.setPixelColor(1, pixels.Color(0, 0, 0));
    pixels.setPixelColor(2, pixels.Color(0, 0, 0));
  }
    pixels.show();
}

void setIndexLEDs(bool value){
  //pixels.clear();

  if(value){
    pixels.setPixelColor(3, pixels.Color(0, 0, 150));
    pixels.setPixelColor(4, pixels.Color(0, 0, 150));
  } else{
    pixels.setPixelColor(3, pixels.Color(0, 0, 0));
    pixels.setPixelColor(4, pixels.Color(0, 0, 0));
  }
  pixels.show();
}

void setStoppedLED(bool on){
  if(on){
    pixels.setPixelColor(8, pixels.Color(150, 0, 0));
    pixels.setPixelColor(9, pixels.Color(150, 0, 0));
    pixels.setPixelColor(10, pixels.Color(150, 0, 0));
    pixels.setPixelColor(11, pixels.Color(150, 0, 0));
    pixels.setPixelColor(12, pixels.Color(150, 0, 0));
    pixels.setPixelColor(13, pixels.Color(150, 0, 0));
    pixels.setPixelColor(14, pixels.Color(150, 0, 0));
    pixels.setPixelColor(15, pixels.Color(150, 0, 0));
  } else{
    pixels.setPixelColor(8, pixels.Color(0, 0, 0));
    pixels.setPixelColor(9, pixels.Color(0, 0, 0));
    pixels.setPixelColor(10, pixels.Color(0, 0, 0));
    pixels.setPixelColor(11, pixels.Color(0, 0, 0));
    pixels.setPixelColor(12, pixels.Color(0, 0, 0));
    pixels.setPixelColor(13, pixels.Color(0, 0, 0));
    pixels.setPixelColor(14, pixels.Color(0, 0, 0));
    pixels.setPixelColor(15, pixels.Color(0, 0, 0));
  }

  pixels.show();
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
