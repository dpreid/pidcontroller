#include <MotorController.h>
#include "ArduinoJson-v6.9.1.h"

// Pins on Motor Driver board.
#define AIN1 2
#define AIN2 4
#define PWMA 3
#define STBY 8

//JSON serialization
#define COMMAND_SIZE 64
StaticJsonDocument<COMMAND_SIZE> doc;
char command[COMMAND_SIZE];

// If motor spins in the opposite direction then you can change this to -1.
const int offset = 1;

Motor motor = Motor(AIN1, AIN2, PWMA, offset, STBY);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
   while (! Serial);
   Serial.println("Input value between 0 to 255");
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0) {
      readSerialJSON();
   }
}

void readSerialJSON(){
  if(Serial.available() > 0){
  
    Serial.readBytesUntil(10, command, COMMAND_SIZE);
    deserializeJson(doc, command);

    const char* cmd = doc["cmd"];

    if(strcmp(cmd,"forward")==0){
      Serial.println("{\"cmd\":\"forward\"}");
      motor.drive(200);
    } 
    else if(strcmp(cmd,"backward")==0){
      Serial.println("{\"cmd\":\"backward\"}");
      motor.drive(-200);
  } 
  else if(strcmp(cmd,"brake") == 0){
    Serial.println("{\"cmd\":\"brake\"}");
    motor.brake();
  }
  }
}
