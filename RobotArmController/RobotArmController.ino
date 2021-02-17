
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
#define ARM_OFFSET 0.0
 
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
volatile int encoder_direction_index = 1;

unsigned long current_time_encoder = 0;
unsigned long previous_time_encoder = 0;
//other user set values
float set_position = 0;     //the position the user has set

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
int position_limit = 300;   //the soft position limit either side of the zero point.
int position_motor_off = 400;   //the maximum position to which the motor will run before switching to AWAITING_STOP
int arm_min = 60;           //the range of positions for the servo connected to the arm
int arm_max = 120;
float zero_error = 10.0;
float zero_signal_CCW = 30;     //the motor signal for zeroing the position.
float zero_signal_CW = 30;
float zero_offset = -200.0;    //the encoderPos difference between index and digger arm 0 position

int pid_interval = 20;       //ms, for timer interrupt

bool encoder_newly_attached = false;
bool timer_interrupt = false;
bool index_interrupt = false;

#define CPU_HZ 48000000
#define TIMER_PRESCALER_DIV 1024
float timer_interrupt_freq = 1000.0/pid_interval;   

//hardware mode switch off timer
float mode_start_time = 0;        //ms
float shutdown_timer = 30000;     //ms
float max_timer = 60000;          //ms

bool moving = true;
int awaiting_stop_lastPos = 0;
int awaiting_stop_thisPos = 0;
int sameCount = 100;   
int sameNeeded = 100;        //number of loops with the same encoder position to assume that motor is stopped.

//for initialisation
int kick_dir = 1;
float kick_magnitude = 100.0;
//bool initial_index_hit = false;

float friction_comp_static_CW = (1.0/12.0)*255;
float friction_comp_static_CCW = (1.0/12.0)*255;
float friction_comp_window = 5.0;
float friction_comp_dynamic_CW = (0.5/12.0)*255;
float friction_comp_dynamic_CCW = (0.5/12.0)*255;

//lowpass filter
volatile float error_position_filter = 0.0;
volatile float previous_error_position_filter = 0.0;
volatile float previous_previous_error_position_filter = 0.0;
volatile float error_speed_filter = 0.0;
volatile float previous_error_speed_filter = 0.0;
volatile float previous_previous_error_speed_filter = 0.0;

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
  STATE_CHANGE_ARM,      //state for extending and retracting digger arm
  STATE_INITIALISE      //robot arm specific state for setting initial direction position
} StateType;

//state Machine function prototypes
//these are the functions that run whilst in each respective state.
void Sm_State_Zero(void);
void Sm_State_Offset(void);
void Sm_State_Awaiting_Stop(void);
void Sm_State_Stopped(void);
void Sm_State_PID_Position(void);
void Sm_State_Change_Arm(void);
void Sm_State_Initialise(void);

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
  {STATE_INITIALISE, Sm_State_Initialise},
};
 
int NUM_STATES = 7;

/**
 * Stores the current state of the state machine
 */
 
StateType SmState = STATE_INITIALISE;    //START IN THE INITIALISE STATE

//DEFINE STATE MACHINE FUNCTIONS================================================================

void Sm_State_Initialise(void){

  servo.write(90 + ARM_OFFSET);
  delay(100);
  
  int extreme_1 = 0;
  int extreme_2 = 0;
  unsigned long previousMillis = millis();
  const long interval = 3000;

  while(millis() <= previousMillis + interval){
      motor.drive(kick_magnitude);
  }
  
  extreme_1 = encoderPos;

  previousMillis = millis();
  while(millis() <= previousMillis + interval){
      motor.drive(-kick_magnitude);
  }
  
  extreme_2 = encoderPos;

  float mid_point = (extreme_1 + extreme_2) / 2.0;
  encoderPos -= mid_point;
    
    
  SmState = STATE_ZERO;
  
}


//TRANSITION: Stopped -> Stopped
void Sm_State_Stopped(void){  

//  if(servo.attached()){
//    servo.detach();
//  }
       
  set_position = encoderPos;   //reset the user set values so that when it re-enters a PID mode it doesn't start instantly
 
  //report_encoder();
  SmState = STATE_STOPPED;
}


void Sm_State_Awaiting_Stop(void){
  motor.brake();
  
    awaiting_stop_lastPos = awaiting_stop_thisPos;
    awaiting_stop_thisPos = encoderPos;
    if (awaiting_stop_thisPos == awaiting_stop_lastPos){
      sameCount = sameCount - 1;
    }
    else{
      sameCount = sameNeeded;
    }
    
      
    if (sameCount <= 0){
      moving = false;
      Serial.println("{\"awaiting_stop\":true}");
       SmState = STATE_STOPPED;    //transition to the stopped state.
    } else{
      SmState = STATE_AWAITING_STOP;
    }

 
}

//TRANSITION: PID_POSITION -> PID_POSITION
void Sm_State_PID_Position(void){

  float friction_comp_static = friction_compensation_static(PID_signal, encoderAngVel, error);
  float friction_comp_dynamic = friction_compensation_dynamic(PID_signal, error);

  float drive_signal = PID_signal + friction_comp_static + friction_comp_dynamic;

    if(drive_signal > 200){
      motor.drive(200);
    } else if(drive_signal < -200){
      motor.drive(-200);
    } else{
      motor.drive(drive_signal);
    }
  
  //report_encoder();
  SmState = STATE_PID_POSITION_MODE;

  if(millis() >= mode_start_time + shutdown_timer){
    SmState = STATE_AWAITING_STOP;
  } else if(abs(encoderPos) > position_motor_off){
    SmState = STATE_AWAITING_STOP;
  }

}

//TRANSITION: ZERO -> AWAITING_STOP
void Sm_State_Zero(void){
//initialisation sets the 0 position of encoder

  if(encoderPos > 0){
     motor.drive(zero_signal_CCW);
     zero_signal_CCW += 0.1;
  } else{
    motor.drive(-zero_signal_CW);
    zero_signal_CW += 0.1;
  }
  
  //report_zero_signal();

  delay(100);
  
  //report_encoder();
  if((encoderPos > zero_error || encoderPos < -zero_error)){    //allowed calibration error
    SmState = STATE_ZERO;  
  } else{
    //index pin has been zeroed so...
    SmState = STATE_AWAITING_STOP;  
  }

  if(millis() >= mode_start_time + shutdown_timer){
    SmState = STATE_AWAITING_STOP;
  }
}

//INDEX PIN IS ALIGNED WITH DIGGER ARM SO NO NEED FOR OFFSET
//TRANSITION: OFFSET -> AWAITING_STOP
void Sm_State_Offset(void){
  //enter this state with the index pin having been zeroed and encoder position set to zero within a zero_error

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
    SmState = STATE_AWAITING_STOP;  
  }
}

//TRANSITION: CHANGE_ARM -> CHANGE_ARM
void Sm_State_Change_Arm(void){

  if(!servo.attached()){
    servo.attach(SERVO);
  }

  //if the servo set position has changed since the last call to write then call write
  float new_position = set_arm_extension + ARM_OFFSET;
  if(servo.read() != new_position){
      servo.write(new_position);
      delay(100);
  }

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
  timer_interrupt = true;
  
  if(SmState == STATE_PID_POSITION_MODE){
     calculatePositionPID();
  } 

  report_encoder();
}

// SETUP AND LOOP==================================================================================
void setup() {
  
  servo.attach(SERVO);
  servo.write(set_arm_extension + ARM_OFFSET);
  delay(100);
  //servo.detach();

  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  pinMode(indexPin, INPUT);

  pixels.begin(); // INITIALIZE NeoPixel
  
  attachEncoderInterrupts();

  float t = millis();
  current_time_encoder = t;
  previous_time_encoder = t;
  previous_report_time = t;
  mode_start_time = t;

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

    mode_start_time = millis();   //on any command sent, reset the start time for hardware switch off
  
    Serial.readBytesUntil(10, command, COMMAND_SIZE);
    deserializeJson(doc, command);
    
    const char* set = doc["set"];

    if(strcmp(set, "position")==0){
      if(SmState == STATE_PID_POSITION_MODE){
        float new_position = doc["to"];
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
  else if(strcmp(set, "arm")==0){
    if(SmState == STATE_CHANGE_ARM){
      float new_position = doc["to"];
      if(new_position >= arm_min && new_position <= arm_max){
        set_arm_extension = new_position;
      } else{
        Serial.println("Outside position range");
      }
    } else{
      Serial.println("In wrong state to set arm");
    }
  }
  else if(strcmp(set, "mode")==0){
      const char* new_mode = doc["to"];

      if(SmState == STATE_STOPPED){
        if(strcmp(new_mode, "positionPid") == 0){
          resetPIDSignal();
          SmState = STATE_PID_POSITION_MODE;
        }
        else if(strcmp(new_mode, "zero") == 0){
          SmState = STATE_ZERO;
        }
        else if(strcmp(new_mode, "changeArm") == 0){
          SmState = STATE_CHANGE_ARM;
        }
        else if(strcmp(new_mode, "initialise") == 0){
          SmState = STATE_INITIALISE;
        }
       
      } else {
        
        if(strcmp(new_mode, "stop") == 0){
          resetPIDSignal();
          SmState = STATE_AWAITING_STOP;      
        } 
        
      }
      
   
    } else if(strcmp(set, "parameters")==0){

      resetPIDSignal();

      if(!doc["kp"].isNull()){
        Kp = doc["kp"];
      }

      if(!doc["ki"].isNull()){
        Ki = doc["ki"];
      }

      if(!doc["kd"].isNull()){
        Kd = doc["kd"];
      }

      if(!doc["dt"].isNull()){
        pid_interval = doc["dt"];
        float timer_interrupt_freq = 1000/pid_interval;  
        setTimerFrequency(timer_interrupt_freq);        
      }
    } 
    
    else if(strcmp(set, "timer") == 0){
      float new_timer = doc["to"];
      new_timer *= 1000.0;
      if(new_timer > 0 && new_timer <= max_timer){
        shutdown_timer = new_timer;
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

  error_position_filter = 0;
  previous_error_position_filter = 0;
  previous_previous_error_position_filter = 0;

  proportional_term = 0.0;
  integral_term = 0.0;
  derivative_term = 0.0;
}


//outputs encoder position to serial bus.
//NEW++++++++++++++++++++ RUNS ON TIMER INTERRUPT
void report_encoder(void)
{
  unsigned long current_time = millis();
 
  detachEncoderInterrupts();
  
  if (encoderPlain){
    Serial.print("position = ");
    Serial.println(encoderPos);
  }
  else{
    Serial.print("{\"enc\":");
    Serial.print(encoderPos);
    Serial.print(",\"time\":");
    Serial.print(current_time); 
    Serial.print(",\"p_sig\":");
    Serial.print(proportional_term);
    Serial.print(",\"i_sig\":");
    Serial.print(integral_term);
    Serial.print(",\"d_sig\":");
    Serial.print(derivative_term);
    Serial.println("}");
}

  attachEncoderInterrupts();
}

void report_zero_signal(){
  Serial.print("zero signal = ");
  Serial.println(zero_signal_CW);
        
}

void attachEncoderInterrupts(void){
  encoder_newly_attached = true;
  attachInterrupt(digitalPinToInterrupt(encoderPinA), doEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), doEncoderB, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(indexPin), doIndexPin, RISING);
}

void detachEncoderInterrupts(void){
  detachInterrupt(digitalPinToInterrupt(encoderPinA));
  detachInterrupt(digitalPinToInterrupt(encoderPinB));
  //detachInterrupt(digitalPinToInterrupt(indexPin));
}

// Interrupt on A changing state
void doEncoderA() {
  
  A_set = digitalRead(encoderPinA) == HIGH;
  // and adjust counter + if A leads B
  encoderPosLast = encoderPos;
  encoderPos += (A_set != B_set) ? +1 : -1;

//ENCODER SPEED NEEDS TO BE CALCULATED FOR FRICTION COMPENSATION
//DIRECTION OF ENCODER IS NOT IMPORTANT SO IS NOT INCLUDED
  if(A_set){
      current_time_encoder = micros();
    if(!encoder_newly_attached){
      if(!timer_interrupt){
        if(!index_interrupt){
          if(current_time_encoder > previous_time_encoder){
            encoderAngVel = 60000000.0/((current_time_encoder - previous_time_encoder)*500);    //rpm
//            previous_time_encoder = current_time_encoder;
            }
        } else{
          index_interrupt = false;
        }
         
      } else{
        timer_interrupt = false;
      }
    } else{
      encoder_newly_attached = false;
    }
  previous_time_encoder = current_time_encoder;
  }


  encoderWrap();


}

// Interrupt on B changing state
void doEncoderB() {
  
  B_set = digitalRead(encoderPinB) == HIGH;
  // and adjust counter + if B follows A
  encoderPosLast = encoderPos;
  encoderPos += (A_set == B_set) ? +1 : -1;
  encoderWrap();

}

//ENCODER DIRECTION IS ONLY NECESSARY FOR CALCULATING ANG VEL, SO ONLY NEEDS TO BE CORRECT WHEN
//INDEX PIN TRIGGERS. Error in direction on wrap is OK....?
void doIndexPin(void){
  index_interrupt = true;
    //get direction of rotation through index position
    if(encoderPos - encoderPosLast >= 0){
      encoder_direction_index = -1;
    } else{
      encoder_direction_index = 1;
    }
    
    led_index_on = !led_index_on;
    setIndexLEDs(led_index_on);

//    if(SmState == STATE_INITIALISE){
//      initial_index_hit = true;
//      encoderPos = 0;   //whilst initialising, when index pin hit set the encoder position to 0.
//      
//    }

      
  
}

void encoderWrap(void){
  if (encoderPos > position_max) {
    encoderPos -= 2*position_max;
    } else if (encoderPos < -position_max) {
      encoderPos += 2*position_max;
      }
}

////DISCRETE TIME VERSION with filter
void calculatePositionPID(void){
    previous_previous_error = previous_error;
    previous_error = error;

    previous_previous_error_position_filter = previous_error_position_filter;
    previous_error_position_filter = error_position_filter;
    
  float error_pos = encoderPos - set_position;
  int dir = error_pos / abs(error_pos);    //should be +1 or -1.
  
  float error_pos_inverse = 2*position_limit - abs(error_pos);

  if(abs(error_pos) <= abs(error_pos_inverse)){
    error = error_pos;
  } else {
    error = -1*dir*error_pos_inverse;
  }
  //convert error to an angular error in deg
  error = error*180.0/position_limit;

  error_position_filter = lowpass_filter(error, previous_error_position_filter);
  
  
  float delta_t = pid_interval/1000.0;
  float Ti = Kp/Ki;
  float Td = Kd/Kp;

  float delta_p = Kp*(error - previous_error);
  proportional_term += delta_p;

  float delta_i = Kp*delta_t*error/Ti;
  integral_term += delta_i;

  float delta_d =  Kp*(Td/delta_t)*(error_position_filter - 2*previous_error_position_filter + previous_previous_error_position_filter);
  derivative_term += delta_d; 
  
 //float new_signal = Kp*(error - previous_error + delta_t*error/Ti +(Td/delta_t)*(error_position_filter - 2*previous_error_position_filter + previous_previous_error_position_filter));
  
  float new_signal = delta_p + delta_i + delta_d;
  
  PID_signal += new_signal;
    

}

float lowpass_filter(float input, float previous_output){
  float a = 0.1;
  return (1-a)*previous_output + a*input;
}

float friction_compensation_static(float drive_signal, float encoderAngVel, float error){
 //static
 if(abs(encoderAngVel) < 5 && abs(drive_signal) > 0){
  if(error > friction_comp_window){
    return friction_comp_static_CCW;
  } else if(error < -friction_comp_window){
    return -friction_comp_static_CW;
  } else if(error > 0){
    return friction_comp_static_CCW * error/friction_comp_window;
  } else {
    return -friction_comp_static_CW * error/friction_comp_window;
  }
 }
  
 
}

//dynamic friction, no window
float friction_compensation_dynamic(float drive_signal, float error){
  if(drive_signal > 0){
      return friction_comp_dynamic_CCW;
 } else if(drive_signal < 0){
      return -friction_comp_dynamic_CW;
 } else {
    return 0.0;
  }
 
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
    pixels.setPixelColor(3, pixels.Color(150, 0, 0));
    pixels.setPixelColor(4, pixels.Color(150, 0, 0));
  } else{
    pixels.setPixelColor(3, pixels.Color(0, 0, 0));
    pixels.setPixelColor(4, pixels.Color(0, 0, 0));
  }

  pixels.show();
}

void setChangingArmLEDs(bool on){
  if(on){
    pixels.setPixelColor(3, pixels.Color(0, 150, 0));
    pixels.setPixelColor(4, pixels.Color(0, 150, 0));
  } else{
    pixels.setPixelColor(3, pixels.Color(0, 0, 0));
    pixels.setPixelColor(4, pixels.Color(0, 0, 0));
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
