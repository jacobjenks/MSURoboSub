#include <Wire.h>
#include "Arduino_I2C_ESC.h"
#include <ctype.h>

int signal;
const int numMotors = 6;
String serialMessage;

enum MotorE{
  MEForwardPort,
  MEForwardStar,
  MEDepthFore,
  MEDepthAft,
  MEStrafeTop,
  MEStrafeBottom
};

Arduino_I2C_ESC motors[numMotors] = {
  Arduino_I2C_ESC(0x2C),//ForwardPort
  Arduino_I2C_ESC(0x2F),//ForwardStar
  Arduino_I2C_ESC(0x2D),//DepthFore
  Arduino_I2C_ESC(0x30),//DepthAft
  Arduino_I2C_ESC(0x2E),//StrafeTop
  Arduino_I2C_ESC(0x2B)//StrafeBottom
};

boolean motorCon[numMotors];//Motor connection status
int motorRPM[numMotors];//Motor RPM
//Array of % thrust values we'd like each motor to operate at
//Values range between -1 and 1 for 100% forward/reverse thrust
float motorCommand[] = {0,0,0,0,0,0};

void setup() {
  Serial.begin(57600);
  Serial.println("Starting");
  
  Wire.begin();

  // Optional: Add these two lines to slow I2C clock to 12.5kHz from 100 kHz
  // This is best for long wire lengths to minimize errors
  //TWBR = 158;  
  //TWSR |= bit (TWPS0);
  
  getMotorStatus();
}

//Check motor connection, return false if any motors fail
boolean getMotorStatus(){
  boolean stat = true, fail = false;
  for(int i = 0; i < numMotors; i++){
    stat = motors[i].isAlive();
    if(stat = false)
      fail = false;
    motorCon[i] = stat;
  }
  return fail;
}

void getMotorRPM(){
  for(int i = 0; i < numMotors; i++)
    motorRPM[i] = (int)motors[i].rpm();
}

//Run actual update function that updates values from motor controllers
//Also call functions to store these values
void motorUpdate(){
  for(int i = 0; i < numMotors; i++)
    motors[i].update();

  getMotorStatus();
  getMotorRPM();
}

//This function converts a value between -1 and 1 to a throttle value
//Actual throttle values are-32767 to 32767
int16_t percentToThrottle(float t){
  return (int16_t)(t * 32767);
}

//Command motors to run at values specified in motorCommand array
void setMotorThrottles(){
  for(int i = 0; i < numMotors; i++)
    motors[i].set(percentToThrottle(motorCommand[i]));
}

//{ForwardPort,ForwardAft,DepthFore,DepthAft,StrafeTop,StrafeBottom,
// Torpedo1,Torpedo2,Dropper1,Dropper2,Arm}
void parseSerialInput(){
  
}

//Make sure serial input string is properly formatted
//String should be list of X numbers separated by commas, and wrapped in {}
boolean validInput(String s){
  if(s.charAt(0) != '{' || s.charAt(s.length() - 1) != '{'){
    //Error output
    return false;
  }
  int currentIndex = 1;
  int numNumbers;//Number of integers separated by commas in string
  for(int i = 1; i < s.length(); i++){
    if(s.charAt(i) == ',' || i == s.length() - 2){
      for(int j = 0; j < i - currentIndex; j++){
        if(!isDigit(s.charAt(j)))
          return false;
      }
      numNumbers++;
    }
  }
  
  if(numNumbers != 11)
    return false;
  
  return true;
}

String buildSerialOutput(){
  
}

void loop() {

  if ( Serial.available() > 0 ) {
    signal = Serial.parseInt();
  }

  motorUpdate();
  delay(250); // Update at roughly 4 hz for the demo
}
