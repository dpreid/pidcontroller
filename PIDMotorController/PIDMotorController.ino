
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
#include <Adafruit_NeoPixel.h>
#include <Stepper.h>
#include <QueueArray.h>
#include <MotorController.h>
#include <MotorControllerPmodHB3.h>

#include "ArduinoJson-v6.9.1.h"

//NeoPixel LED setup
#define NEOPIXEL_PIN 14
#define NUMPIXELS 64
Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
// Pins on Motor Driver board.
#define AIN1 5
#define AIN2 4
#define PWMA 6
#define STBY 8
//Pins for Stepper motor - governor control
//#define SIN1 7
//#define SIN2 19       //motors wired up incorrectly??
//#define SIN3 12
//#define SIN4 18
#define SDIR 9
#define SSTP 12
#define SEN 16

#define encoderPinA 2     //these pins all have interrupts on them.
#define encoderPinB 3
#define indexPin 11

//LED output pins         //SWAPPING THESE OUT FOR NEOPIXELS
//#define ledPositive 14
//#define ledIndex 15
//#define ledNegative 16
//#define ledPowerOn 17

#define limitSwitchLower 19
#define limitSwitchUpper 21

bool debug = false;
unsigned long report_interval = 10;
bool encoderPlain = false;

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
float Kp = 0.5;               //PID parameters
float Ki = 0;
float Kd = 0;

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

//Motor motor = Motor(AIN1, AIN2, PWMA, offset, STBY);    //CHOICE FOR WHICH MOTOR DRIVER TO USE
MotorHB3 motor = MotorHB3(AIN1, PWMA, offset);

//stepper variables====================
volatile float governor_pos = 0;
volatile float set_governor_pos = 0;
float max_governor_pos = 100;
//int steps_in_one_rotation = 513;   //32 actual steps * 16.032 internal gearing PROTOTYPE MOTOR
int steps_in_one_rotation = 800;   //prototype motor 2
float dist_one_rotation = 0.5;    //mm increase in height of lower governor position
long stepper_speed = 100;
//Stepper stepper = Stepper(steps_in_one_rotation, SIN1, SIN2, SIN3, SIN4);
Stepper stepper = Stepper(steps_in_one_rotation, SDIR, SSTP);
//======================================

int speed_limit = 100;        //pin signal limit equivalent to approx 5V using 12V power supply.
int position_limit = 1000;    //the number of encoderPos intervals in half a rotation (encoder rotates from -1000 to 1000).

int pid_interval = 3;       //ms, for timer interrupt

int sameNeeded = 100;        //number of loops with the same encoder position to assume that motor is stopped.

bool doInterruptAB = false;
bool doInterruptIndex = false;

bool lowerLimitReached = false;

#define CPU_HZ 48000000
#define TIMER_PRESCALER_DIV 1024
float timer_interrupt_freq = 1000/pid_interval;   //200Hz once every 5ms
/**
 * Defines the valid states for the state machine
 * 
 */
typedef enum
{
  STATE_CALIBRATE,      //state for resetting the encoder 0 position to the index point
  STATE_AWAITING_STOP,  //checking if the motor has stopped
  STATE_STOPPED,        //no drive to motor
  STATE_PID_SPEED_MODE, //pid controller mode - 1st order, speed functions
  STATE_PID_POSITION_MODE,  //pid controller - 2nd order, position functions
  STATE_DC_MOTOR_MODE,    //regular dc motor functions with no PID controller
  STATE_CONFIGURE, //mode for changing position of the governor.
} StateType;

//state Machine function prototypes
//these are the functions that run whilst in each respective state.
void Sm_State_Start_Calibration(void);
void Sm_State_Awaiting_Stop(void);
void Sm_State_Stopped(void);
void Sm_State_PID_Speed(void);
void Sm_State_PID_Position(void);
void Sm_State_DC_Motor(void);
void Sm_State_Configure(void);

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
  {STATE_CONFIGURE, Sm_State_Configure}
};
 
int NUM_STATES = 7;

/**
 * Stores the current state of the state machine
 */
 
StateType SmState = STATE_STOPPED;    //START IN THE STOPPED STATE

//DEFINE STATE MACHINE FUNCTIONS================================================================


void Sm_State_Stopped(void){
  //motor.brake();
  enableStepper(false);   
  //doInterruptAB = false;
  doInterruptIndex = true;       
  lowerLimitReached = false;    //CHECK THIS
  set_position = encoderPos;   //reset the user set values so that when it re-enters a PID mode it doesn't start instantly
  set_speed = 0;
  encoderAngVel = 0;
  //report_encoder_speed();
  report_encoder();
  SmState = STATE_STOPPED;
}

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

void Sm_State_PID_Position(void){
    //doInterruptAB = true;
    doInterruptIndex = false;

//PID position doesn't need a speed limit

//   if(PID_signal >=-speed_limit && PID_signal <= speed_limit){  
//      motor.drive(PID_signal); 
//   } else if(PID_signal > speed_limit){
//      motor.drive(speed_limit);
//   } else{
//      motor.drive(-speed_limit);
//   }

  motor.drive(PID_signal);

   //setRotationLEDs(PID_signal);
  
  report_encoder();
  SmState = STATE_PID_POSITION_MODE;

}

void Sm_State_PID_Speed(void){

    //doInterruptAB = false;
    doInterruptIndex = true;
    
//limit the rotation speed for now!
//   if(PID_signal >=-speed_limit && PID_signal <= speed_limit){  
//      motor.drive(PID_signal); 
//   } else if(PID_signal > speed_limit){
//      motor.drive(speed_limit);
//   } else{
//      motor.drive(-speed_limit);
//   }
    motor.drive(PID_signal);
  
  report_encoder();  
  //report_encoder_speed();
  SmState = STATE_PID_SPEED_MODE;
}


void Sm_State_Start_Calibration(void){
  
  //doInterruptAB = false;
  doInterruptIndex = true;

  bool index_state = led_index_on;

  while(index_state == led_index_on){
    motor.drive(-encoder_direction * 60);
  }

//  int t = millis();
//  while(millis() < t + 100){
//    motor.brake();  
//  }
  motor.brake();

  encoderPos = 0;

  delay(100);
  
  report_encoder();
  if(encoderPos > 20 || encoderPos < -20){    //allowed calibration error
    SmState = STATE_CALIBRATE;  
  } else{
    enableStepper(true);
    while(!lowerLimitReached){
      stepper.setSpeed(stepper_speed);
      stepper.step(1);
      //delay(100);
    }
//    stepper.setSpeed(stepper_speed);
//    stepper.step(-100);
    //lowerLimitReached = false;
    SmState = STATE_STOPPED;  
  }
  
}

void Sm_State_DC_Motor(void){
  
  if(set_speed <= speed_limit && set_speed >= -speed_limit){
    motor.drive(set_speed);  
  } else if(set_speed > speed_limit){
    motor.drive(speed_limit);
  } else{
    motor.drive(-speed_limit);
  }
  

  //doInterruptAB = false;
  doInterruptIndex = true;
  
  //setRotationLEDs(encoderAngVel);
  report_encoder();
  //report_encoder_speed();
  SmState = STATE_DC_MOTOR_MODE;
}

//User sets the governor position that they want. Stepper steps until that position is reached. Limit switch
//can stop the motion.
void Sm_State_Configure(void){
  enableStepper(true);
  //digitalWrite(SEN, LOW);   //enable the stepper
  stepper.setSpeed(stepper_speed);
  
  float diff = set_governor_pos - governor_pos;
  //float num_steps_to_take = getStepsFromDistance(diff);

  Serial.print("gov pos = ");
  Serial.println(governor_pos);
  
  if(diff > 0){ 
    Serial.println("moving up");
    //stepper.step(num_steps_to_take);
    stepper.step(-1);
    governor_pos += dist_one_rotation / steps_in_one_rotation;
    
  } else if(diff < 0){
    Serial.println("moving down");
    //stepper.step(num_steps_to_take);
    stepper.step(1);
    governor_pos -= dist_one_rotation / steps_in_one_rotation;
    
  } 
    //governor_pos = set_governor_pos;
  if(diff <= 0.001 && diff >= -0.001) { //need to test this error allowance
    stepper.setSpeed(0);
    SmState = STATE_STOPPED;
  } else{
    SmState = STATE_CONFIGURE;
  }
  //SmState = STATE_STOPPED;
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
  } else if(SmState == STATE_PID_SPEED_MODE){
    calculateSpeedPID();
  } 
}

// SETUP AND LOOP==================================================================================
void setup() {
  //setup encoder pins
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  pinMode(indexPin, INPUT_PULLUP);
  pinMode(SEN, OUTPUT);
  enableStepper(false);
  //digitalWrite(SEN, HIGH);    //start stepper in disabled mode

  
  //digitalWrite(encoderPinA, HIGH);  // turn on pull-up resistor....DONE ABOVE
  //digitalWrite(encoderPinB, HIGH);  // turn on pull-up resistor
  //digitalWrite(indexPin, HIGH);

  //setup stepper motor pins
//  pinMode(SIN1, OUTPUT);
//  pinMode(SIN2, OUTPUT);
//  pinMode(SIN3, OUTPUT);
//  pinMode(SIN4, OUTPUT);
  
  //setup led output pins
//  pinMode(ledPositive, OUTPUT);
//  pinMode(ledIndex, OUTPUT);
//  pinMode(ledNegative, OUTPUT);
//  pinMode(ledPowerOn, OUTPUT);

  pixels.begin(); // INITIALIZE NeoPixel

  //setup limit switches
  pinMode(limitSwitchLower, INPUT_PULLUP);
  //pinMode(limitSwitchUpper, INPUT_PULLUP);
  
  //digitalWrite(ledPowerOn, HIGH);   //just to show that arduino is on.
  
  // encoder pin on interrupt (pin 2)
  attachInterrupt(digitalPinToInterrupt(encoderPinA), doEncoderA, CHANGE);
  // encoder pin on interrupt (pin 3)
  attachInterrupt(digitalPinToInterrupt(encoderPinB), doEncoderB, CHANGE);
  // encoder pin on interrupt (pin 11)
  attachInterrupt(digitalPinToInterrupt(indexPin), doIndexPin, RISING);

  //interruptsfor limit switches
  attachInterrupt(digitalPinToInterrupt(limitSwitchLower), doLimitLower, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(limitSwitchUpper), doLimitUpper, HIGH);   

  current_time_index = millis();   
  previous_time_index = millis();

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
        int new_position = doc["param"];
        if(new_position >= -position_limit && new_position <= position_limit){
          set_position = new_position;
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

      if(SmState == STATE_STOPPED){
        
        if(strcmp(new_mode, "PID_SPEED_MODE") == 0){
        SmState = STATE_PID_SPEED_MODE;
        } 
        else if(strcmp(new_mode, "PID_POSITION_MODE") == 0){
          SmState = STATE_PID_POSITION_MODE;
        }
        else if(strcmp(new_mode, "DC_MOTOR_MODE") == 0){
          SmState = STATE_DC_MOTOR_MODE;
        }
        else if(strcmp(new_mode, "CONFIGURE") == 0){
          Serial.println("Entering configuration state");
          SmState = STATE_CONFIGURE;
        }
        else if(strcmp(new_mode, "CALIBRATE") == 0){
          SmState = STATE_CALIBRATE;
        }
        
      } else {
        
        if(strcmp(new_mode, "STOP") == 0){
          SmState = STATE_AWAITING_STOP;          
        } 
        
      }
      
   
    } else if(strcmp(cmd, "set_parameters")==0){

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
      }

      if(!doc["N_errors"].isNull()){
        numErrors = doc["N_errors"];
      }
    } else if(strcmp(cmd, "set_height") == 0){
      float new_pos = doc["param"];
      if(new_pos <= max_governor_pos && new_pos >= 0){
        Serial.println("new governor position set");
        set_governor_pos = new_pos;
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
        Serial.print(millis());  
        Serial.println("}");
      }
    }
  
}

//outputs encoder speed to serial bus.
void report_encoder_speed(void){
   if (millis() % report_interval == 0){
      if (encoderPlain){
        Serial.print("ang vel = ");
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
//CURRENTLY ALWAYS DOING ENCODER INTERRUPTS IN ORDER TO GET DIRECTION
void doEncoderA() {
  A_set = digitalRead(encoderPinA) == HIGH;
  // and adjust counter + if A leads B
  encoderPosLast = encoderPos;
  encoderPos += (A_set != B_set) ? +1 : -1;
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
    setIndexLEDs(led_index_on);
    //digitalWrite(ledIndex, led_index_on);
    if(encoder_direction / encoder_direction_last < 0){
      setRotationLEDs(encoderAngVel);  
    }
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
//  if(value > 0){
//    digitalWrite(ledPositive, HIGH);
//    digitalWrite(ledNegative, LOW);
//  } else if(value < 0){
//    digitalWrite(ledPositive, LOW);
//    digitalWrite(ledNegative, HIGH);
//  } else{
//    digitalWrite(ledPositive, LOW);
//    digitalWrite(ledNegative, LOW);
//  }

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

//if limit switch hit then stepper should move up.
void doLimitLower(void){
  Serial.println("lower limit reached");
    lowerLimitReached = true;
    //digitalWrite(SEN, LOW);   //enable stepper
    enableStepper(true);
    stepper.setSpeed(stepper_speed);    
    stepper.step(-800);
    governor_pos = 0;       //this is our zero point
    set_governor_pos = 0;
//
    SmState = STATE_STOPPED;

}

void doLimitUpper(void){
  Serial.println("upper limit reached");
}

//RETURN: the number of steps required to move the governor a vertical distance (in mm) of dist_mm.
//INPUT: The distance in mm that the governor should move. + upwards; - downwards
float getStepsFromDistance(float dist_mm){
  float num_full_turns = dist_mm / dist_one_rotation;  //dist in mm
  float total_steps = num_full_turns*steps_in_one_rotation;

  return total_steps;
}

void enableStepper(bool on){
  if(on){
    digitalWrite(SEN, LOW);
  } else{
    digitalWrite(SEN, HIGH);
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
