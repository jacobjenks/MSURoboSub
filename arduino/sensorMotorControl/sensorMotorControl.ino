#include <Wire.h>
#include "Arduino_I2C_ESC.h"
#include <ctype.h>
#include <ros.h>


int signal;
const int numMotors = 6;
String serialMessage;

ros::NodeHandle nh;
std_msgs:: motorStatusMsg;
ros::Publisher pubMotorStatus("motor_status", &motorStatusMsg);
std_msgs:: motorRPMMsg;
ros::Publisher pubMotorRpm("motor_rpm", &motorRPMMsgs);
std_msgs::Float32 depthMsg;
ros::Publisher pubDepth("depth", &depthMsg);
std_msgs:: hydroMsg;
ros::Publisher pubHydro("hydrophone", &hdryMsg)

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
float maxThrust = .75;//Percent value indicating what power level we consider to be max thrust

void motorCommandCallback(const std_msgs::Empty& command){
//route command to motor
//msg type needs to be something other than Empty, but not sure what it should be yet
//msg should contain motor ID and thrust setting
}

ros::Subscriber<std_msgs::Empty> commandSub("motorCommand", &motorCommandCB);

void setup() {
  Wire.begin();
  nh.initNode();
  nh.advertise(depth);

  // Optional: Add these two lines to slow I2C clock to 12.5kHz from 100 kHz
  // This is best for long wire lengths to minimize errors
  //TWBR = 158;  
  //TWSR |= bit (TWPS0);
  
  getMotorStatus();
}

//Check motor connection, return false if any motors fail
void getMotorStatus(){
  boolean stat = true, fail = false;
  for(int i = 0; i < numMotors; i++){
    motorStatusMsg.data = motors[i].isAlive();
    pubMotorStatus.publish(&motorStatusMsg);
  }
}

void getMotorRPM(){
  for(int i = 0; i < numMotors; i++){
    motorRPMMsg.data = (int)motors[i].rpm();
    pubMotorRPM.publish(&motorRPMMsg); 
  }
}

//Run actual update function that updates values from motor controllers
//Also call functions to report these values
void motorUpdate(){
  for(int i = 0; i < numMotors; i++)
    motors[i].update();

  getMotorStatus();
  getMotorRPM();
}

//This function converts a value between -1 and 1 to a throttle value
//Actual throttle values are -32767 to 32767
int16_t percentToThrottle(float t){
  return (int16_t)(t * 32767) * maxThrust;
}

//Command motors to run at values specified in motorCommand array
void setMotorThrottles(){
  for(int i = 0; i < numMotors; i++)
    motors[i].set(percentToThrottle(motorCommand[i]));
}

void sensorUpdate(){
  int hydrophone_sensor_pin1= analogRead(hydroPin1);
  int hydrophone_sensor_pin2= analogRead(hydroPin2);
  int hydrophone_sensor_pin3= analogRead(hydroPin3);

  //Calculate hydrophone heading and distance
  hydroMsg = ???; 
  pubHydro.publish(&hydroMsg);

  depthMsg = analogRead(depthPin);
  pubDepth.publish(&depthMsg);
}

void loop() {
  motorUpdate();
  sensorUpdate();
  nh.spinOnce();
  delay(1000); // Update at roughly 4 hz
}
