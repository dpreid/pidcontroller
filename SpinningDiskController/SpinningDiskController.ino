
/*
 * Remote Lab: Spinning Disk Controller
 * David Reid
 * 02/12/20
 * 
 * Based upon Tim Drysdale's penduino code.
 * https://github.com/timdrysdale/penduino
 * 
 * Timer interrupt functions from https://github.com/nebs/arduino-zero-timer-demo/
 */
#include <Adafruit_NeoPixel.h>
#include <MotorControllerPmodHB3.h>

#include "ArduinoJson-v6.9.1.h"


//NeoPixel LED setup
#define NEOPIXEL_PIN 14
#define NUMPIXELS 8
Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
// Pins on Motor Driver board.
#define AIN1 5
#define PWMA 6
 
#define encoderPinA 3     //these pins all have interrupts on them.
#define encoderPinB 2
#define indexPin 11

unsigned long dta; // moving average, needs right shifting by 3 bits to get correct value
bool do_report_encoder = false;
bool do_calculate_position = false;
volatile float speed_angular_velocity = 0; //for speed mode velocity reporting
unsigned long speed_current_time_encoder = 0;
unsigned long speed_previous_time_encoder = 0;

bool debug = false;
//unsigned long report_interval = 10;   //ms
//unsigned long previous_report_time = 0;
bool encoderPlain = false;

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

//other user set values
float set_position = 0;     //the position the user has set
float set_speed = 0;        //user set position for PID_Speed mode and DC_Motor mode
//PID parameters
float Kp = 1.0;              
float Ki = 0.0;
float Kd = 0.0;

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

float position_limit = 250.0;    //the number of encoderPos intervals in half a rotation (encoder rotates from -1000 to 1000).
float zero_error = 10;
float max_rpm = 4000;

float pid_interval = 20.0;       //ms, for timer interrupt    !!!!!!*********************************
int report_integer = 1;          //an integer multiple of the pid_interval for reporting data set to 5 for speed modes, 1 for position mode.
int report_count = 0;

bool timer_interrupt = false;
bool index_interrupt = false;

#define CPU_HZ 48000000
#define TIMER_PRESCALER_DIV 1024
float timer_interrupt_freq = 1000.0/pid_interval;   

//hardware mode switch off timer
float mode_start_time = 0;        //ms
float shutdown_timer = 120000;     //ms was 30sec
float max_timer = 120000;          //ms was 60sec

bool moving = true;
int awaiting_stop_lastPos = 0;
int awaiting_stop_thisPos = 0;
int sameCount = 100;   
int sameNeeded = 100;        //number of loops with the same encoder position to assume that motor is stopped.

bool friction_comp_on = true;
float friction_comp_static_CW = (1.8/12.0)*255;
float friction_comp_static_CCW = (1.7/12.0)*255;
float friction_comp_window = 5.0;
float friction_comp_dynamic_CW = (1.0/12.0)*255;
float friction_comp_dynamic_CCW = (0.9/12.0)*255;

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
  STATE_AWAITING_STOP,  //checking if the motor has stopped
  STATE_STOPPED,        //no drive to motor
  STATE_PID_SPEED_MODE, //pid controller mode - 1st order, speed functions
  STATE_PID_POSITION_MODE,  //pid controller - 2nd order, position functions
  STATE_DC_MOTOR_MODE,    //regular dc motor functions with no PID controller
} StateType;

//state Machine function prototypes
//these are the functions that run whilst in each respective state.
void Sm_State_Awaiting_Stop(void);
void Sm_State_Stopped(void);
void Sm_State_PID_Speed(void);
void Sm_State_PID_Position(void);
void Sm_State_DC_Motor(void);

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
  {STATE_AWAITING_STOP, Sm_State_Awaiting_Stop},
  {STATE_STOPPED, Sm_State_Stopped},
  {STATE_PID_SPEED_MODE, Sm_State_PID_Speed},
  {STATE_PID_POSITION_MODE, Sm_State_PID_Position},
  {STATE_DC_MOTOR_MODE, Sm_State_DC_Motor}
};
 
int NUM_STATES = 5;

/**
 * Stores the current state of the state machine
 */
 
StateType SmState = STATE_STOPPED;    //START IN THE STOPPED STATE

//DEFINE STATE MACHINE FUNCTIONS================================================================


void Sm_State_Stopped(void){        

  set_position = encoderPos;
  set_speed = 0;
  encoderAngVel = 0;

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

void Sm_State_PID_Speed(void){

  float friction_comp_static = friction_compensation_static(PID_signal, encoderAngVel, error_speed);
  float friction_comp_dynamic = friction_compensation_dynamic(PID_signal, encoderAngVel, error_speed);

  float drive_signal = PID_signal;
	
  if (friction_comp_on) {
	drive_signal += friction_comp_static + friction_comp_dynamic;
  
    if(drive_signal > 200){
      motor.drive(200);
    } else if(drive_signal < -200){
      motor.drive(-200);
    } else{
      motor.drive(drive_signal);
    }

  } else {
	  motor.drive(drive_signal);
	}	
  
  SmState = STATE_PID_SPEED_MODE;

  if(millis() >= mode_start_time + shutdown_timer && set_speed != 0){
    SmState = STATE_AWAITING_STOP;
  }
}

//TRANSITION: PID_POSITION -> PID_POSITION
void Sm_State_PID_Position(void){

  float friction_comp_static = friction_compensation_static(PID_signal, encoderAngVel, error);
  float friction_comp_dynamic = friction_compensation_dynamic(PID_signal, encoderAngVel, error);

  float drive_signal = PID_signal;
   if (friction_comp_on) {
	drive_signal += friction_comp_static + friction_comp_dynamic;
  
    if(drive_signal > 200){
      motor.drive(200);
    } else if(drive_signal < -200){
      motor.drive(-200);
    } else{
      motor.drive(drive_signal);
    }

  } else {
	  motor.drive(drive_signal);
	}	

  SmState = STATE_PID_POSITION_MODE;

  if(millis() >= mode_start_time + shutdown_timer){
    SmState = STATE_AWAITING_STOP;
  }
  // old wrap test was here

}


void Sm_State_DC_Motor(void){
  float drive_signal = set_speed*1.275;

  motor.drive(drive_signal);     //max signal = 127.5 (6V/12V * 255)

  SmState = STATE_DC_MOTOR_MODE;

  if(millis() >= mode_start_time + shutdown_timer && set_speed != 0){
    SmState = STATE_AWAITING_STOP;
  }
}

//*****************************************************************************

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

  do_calculate_position = true;

  report_count++;
  if(report_count >= report_integer){
    do_report_encoder = true;       //NEW ++++++++++++++++++++++++++ report encoder on timer interrupt at pid_interval period.
    report_count = 0;
  }
  
}

// SETUP AND LOOP==================================================================================
void setup() {
  //setup encoder pins, pullup resistors on PCB
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  pinMode(indexPin, INPUT);

  pixels.begin(); // INITIALIZE NeoPixel
  
  attachEncoderInterrupts();

  //initialise time values
  float t = millis();
  current_time_encoder = t;
  previous_time_encoder = t;
  current_time_index = t;   
  previous_time_index = t;
  //previous_report_time = t;
  mode_start_time = t;

  Serial.setTimeout(50);
  Serial.begin(57600);

  startTimer(timer_interrupt_freq);   //setup and start the timer interrupt functions for PID calculations

   while (! Serial);

   
}

void loop() {
  // check flags here for long tasks triggered by interrupts
  if (do_report_encoder) {
	report_encoder();
	do_report_encoder = false;
  }
  if (do_calculate_position){
	if(SmState == STATE_PID_SPEED_MODE){
	  calculateSpeedPID();
	} else if(SmState == STATE_PID_POSITION_MODE){
	  calculatePositionPID();
	}
	do_calculate_position = false;
  }
  Sm_Run();  
}


StateType readSerialJSON(StateType SmState){
  if(Serial.available() > 0){

    mode_start_time = millis();   //on any command sent, reset the start time for hardware switch off
  
    Serial.readBytesUntil(10, command, COMMAND_SIZE);
    deserializeJson(doc, command);
    
    const char* set = doc["set"];

    if(strcmp(set, "speed")==0){
      float new_speed = doc["to"];
      
      if(SmState == STATE_PID_SPEED_MODE){
     
        if(new_speed <= max_rpm && new_speed >= -max_rpm){
          resetPIDSignal();
          set_speed = new_speed;
        }
        
      } else if(SmState == STATE_DC_MOTOR_MODE){
        
        if(new_speed >= -100 && new_speed <= 100){
          set_speed = new_speed;
        }
        
      }
      else{
        Serial.println("In wrong state to set speed");
      }
      
    }  
    else if(strcmp(set, "position")==0){
      if(SmState == STATE_PID_POSITION_MODE){
        float new_position = doc["to"];
		new_position /= 4; //TDD 2021-02-18 reduced from 2000 ppr to 500
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
    else if(strcmp(set, "mode")==0){
      
      const char* new_mode = doc["to"];

      if(SmState == STATE_STOPPED){
        
        if(strcmp(new_mode, "speedPid") == 0){
          report_integer = 5; 
          resetPIDSignal();
          SmState = STATE_PID_SPEED_MODE;
        } 
        else if(strcmp(new_mode, "speedRaw") == 0){
          report_integer = 5;
          SmState = STATE_DC_MOTOR_MODE;
        }
        else if(strcmp(new_mode, "positionPid") == 0){
          report_integer = 1;
          resetPIDSignal();
          SmState = STATE_PID_POSITION_MODE;
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


//outputs encoder position and ang vel to serial bus.
//NEW++++++++++++++++++++ RUNS ON TIMER INTERRUPT
void report_encoder(void)
{

  if (encoderPlain){
    Serial.print("position = ");
    Serial.println(encoderPos*4); //TDD 2021-02-18 mimic that we still have 2000ppr (we only have 500ppr now)
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
    Serial.print(",\"p_sig\":");
    Serial.print(proportional_term);
    Serial.print(",\"i_sig\":");
    Serial.print(integral_term);
    Serial.print(",\"d_sig\":");
    Serial.print(derivative_term);
	Serial.print(",\"sp\":");
	Serial.print(set_position);
	Serial.print(",\"sv\":");
	Serial.print(set_speed);	
    Serial.println("}");
	}
}

void detachEncoderInterrupts(void){
  detachInterrupt(digitalPinToInterrupt(encoderPinA));
  //detachInterrupt(digitalPinToInterrupt(indexPin));
}

void attachEncoderInterrupts(void){
  attachInterrupt(digitalPinToInterrupt(encoderPinA), doEncoderA, RISING);
  //attachInterrupt(digitalPinToInterrupt(indexPin), doIndexPin, RISING);
  previous_time_encoder = micros();
}

// Interrupt on encoder A changing state
// calculate position, direction and speed
void doEncoderA() {
  previous_time_encoder = current_time_encoder;
  current_time_encoder = micros();
  unsigned long dt = current_time_encoder - previous_time_encoder; 
  if (dt > 0 ) { //not overflow
	encoderAngVel = 60e6 / (dt * 500);
  }
  A_set = digitalRead(encoderPinA) == HIGH;
  B_set = digitalRead(encoderPinB) == HIGH;
  // adjust counter + if A leads B
  encoder_direction = (A_set != B_set) ? +1 : -1;
  encoderPos += encoder_direction;

  if (encoderPos > 249) {
	encoderPos = -250;
  }

  if (encoderPos < -250){
	encoderPos = 249;
  }

  if (encoderPos == 0) { // we can't skip this because increments are by one
	speed_current_time_encoder = micros();
	unsigned long speed_dt = speed_current_time_encoder - speed_previous_time_encoder;
	
	if (speed_dt > 0 ) { //not overflow
	  speed_angular_velocity =  60e6 / (speed_dt); //rpm 
	  }
	/*
	Serial.print("{\"enc_ang_vel\":");
	Serial.print(encoderAngVel);
	Serial.print(",\"time\":");
    Serial.print(millis()); 
	Serial.println("}");  
	*/
	speed_previous_time_encoder = speed_current_time_encoder; 
  }
}


//ENCODER DIRECTION IS ONLY NECESSARY FOR CALCULATING ANG VEL, SO ONLY NEEDS TO BE CORRECT WHEN
//INDEX PIN TRIGGERS. Error in direction on wrap is OK....?
void doIndexPin(void){
    index_interrupt = true;
    //get direction of rotation
    
    if(encoderPos - encoderPosLast >= 0){
      encoder_direction_index = -1;
    } else{
      encoder_direction_index = 1;
    }
    

    led_index_on = !led_index_on;
    setIndexLEDs(led_index_on);
    //digitalWrite(ledIndex, led_index_on);
    if(encoder_direction_index / encoder_direction_last < 0){
      setRotationLEDs(encoderAngVel);  
    }

    encoder_direction_last = encoder_direction_index;

//
//    current_time_index = micros();
//    if(encoderAngVel > 2000){
//        if(current_time_index > previous_time_index){
//            encoderAngVel = encoder_direction_index * 60000000.0/((current_time_index - previous_time_index));    //rpm
//          }
//    }
//    previous_time_index = current_time_index;
  
  
}


//DISCRETE TIME VERSION, with filter
void calculateSpeedPID(void){
    previous_previous_error_speed = previous_error_speed;
    previous_error_speed = error_speed;

    previous_previous_error_speed_filter = previous_error_speed_filter;
    previous_error_speed_filter = error_speed_filter;

    error_speed = (set_speed - encoderAngVel)/100.0;

    error_speed_filter = lowpass_filter(error_speed, previous_error_speed_filter);
  
  float delta_t = pid_interval/1000.0;
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
  
  float new_signal = delta_p + delta_i + delta_d;
  
  PID_signal += new_signal;
    

}



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
float friction_compensation_dynamic(float drive_signal, float encoderAngVel, float error){
  if(drive_signal > 0){
      return friction_comp_dynamic_CCW;
 } else if(drive_signal < 0){
      return -friction_comp_dynamic_CW;
 } else {
    return 0.0;
  }
 
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
