#include <Wire.h>
#include "Arduino_I2C_ESC.h"

Arduino_I2C_ESC motForwardPort(0x2C);
Arduino_I2C_ESC motForwardStar(0x2F);
Arduino_I2C_ESC motDepthFore(0x2D);
Arduino_I2C_ESC motDepthAft(0x30);
Arduino_I2C_ESC motStrafeTop(0x2E);
Arduino_I2C_ESC motStrafeBottom(0x2B);

//ME for Motor Enum
enum MotorE{
  MEForwardPort,
  MEForwardStar,
  MEDepthFore,
  MEDepthAft,
  MEStrafeTop,
  MEStrafeBottom
};

int signal;
const int numMotors = 6;
String serialMessage;

Arduino_I2C_ESC motors[numMotors];//Motor objects
boolean motorCon[numMotors];//Motor connection status
int motorRPM[numMotors];//Motor RPM
//Array of % thrust values we'd like each motor to operate at
//Values range between -1 and 1 for 100% forward/reverse thrust
float motorCommand[numMotors];

void setup() {
  Serial.begin(57600);
  Serial.println("Starting");
  
  Wire.begin();

  // Optional: Add these two lines to slow I2C clock to 12.5kHz from 100 kHz
  // This is best for long wire lengths to minimize errors
  //TWBR = 158;  
  //TWSR |= bit (TWPS0);
  
  initMotors();
}

boolean initMotors(){
  //Populate motor array with motor objects
  motors[MEForwardPort] = motForwardPort;
  motors[MEForwardStar] = motForwardStar;
  motors[MEDepthFore] = motDepthFore;
  motors[MEDepthAft] = motDepthAft;
  motors[MEStrafeTop] = motStrafeTop;
  motors[MEStrafeBottom] = mottrafeBottom;
  
  //Initialize all motors to no thrust
  for(int i = 0; i < numMotors; i++){
    motorCommand[i] = 0;
  }
  
  getMotorStatus();
}

//Check motor connection, return false if any motors fail
boolean getMotorStatus(){
  boolean stat = true, fail = false;
  for(int i = 0; i < numMotors; i++){
    stat = motor[i].isAlive();
    if(stat = false)
      fail = false;
    motorCon[i] = stat;
  }
  return fail;
}

void getMotorRPM(){
  for(int i = 0; i < numMotors; i++)
    motorRPM[i] = (int)motor[i].rpm();
}

//Run actual update function that updates values from motor controllers
//Also call functions to store these values
void motorUpdate(){
  for(int i = 0; i < numMotors; i++)
    motor[i].update();

  getMotorStatus();
  getMotorRPM();
}

//This function converts a value between -1 and 1 to a throttle value
//Actual throttle values are-32767 to 32767
int16_t percentToThrottle(float t){
  return (int16_t)(t * 32767);
}

//Command motors to run at values specified in motorCommand array
void motorCommand(){
  for(int i = 0; i < numMotors; i++)
    motor[i].set(percentToThrottle(motorCommand[i]));
}

//{ForwardPort, ForwardAft, DepthFore, DepthAft, StrafeTop, StrafeBottom,
// Torpedo1, Torpedo2, Dropper1, Dropper2, Arm
void parseSerialInput(){
  
}

string buildSerialOutput(){
  
}

void loop() {

  if ( Serial.available() > 0 ) {
    signal = Serial.parseInt();
  }

  motorUpdate();
  delay(250); // Update at roughly 4 hz for the demo
}
