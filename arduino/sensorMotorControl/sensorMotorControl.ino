#include <Wire.h>
#include "Arduino_I2C_ESC.h"
#include <ctype.h>
#include <ros.h>
#include <msurobosub/MotorStatus.h>
#include <msurobosub/Hydro.h>
#include <std_msgs/Float32.h>
#include <msurobosub/MotorCommand.h>
#include <msurobosub/PneumaticCommand.h>


const int numMotors = 6;
const int hydroPin1 = A1;
const int hydroPin2 = A2;
const int hydroPin3 = A3;
const int depthPin = A4;

ros::NodeHandle nh;
msurobosub::MotorStatus motorStatusMsg;
ros::Publisher pubMotorStatus("motorStatus", &motorStatusMsg);
std_msgs::Float32 depthMsg;
ros::Publisher pubDepth("depth", &depthMsg);
msurobosub::Hydro hydroMsg;
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

float maxThrust = .75;//Percent value indicating what power level we consider to be max thrust

void motorCommandCallback(const msurobosub::MotorCommand& command){
  motors[command.motor_id] = command.power;
}

void pneumaticCommandCallback(const msurobosub::PneumaticCommand& command){

}

ros::Subscriber<msurobosub::MotorCommand> subMotorCommand("motorCommand", &motorCommandCallback);
ros::Subscriber<msurobosub::PneumaticCommand> subPneumaticCommand("pneumaticCommand", &pneumaticCommandCallback);

void setup() {
  Wire.begin();
  nh.initNode();
  nh.advertise(pubMotorStatus);
  nh.advertise(pubHydro);
  nh.advertise(pubDepth);
  nh.subscribe(subMotorCommand);
  nh.subscribe(subPneumaticCommand);

  // Optional: Add these two lines to slow I2C clock to 12.5kHz from 100 kHz
  // This is best for long wire lengths to minimize errors
  //TWBR = 158;  
  //TWSR |= bit (TWPS0);
}


//Run actual update function that updates values from motor controllers
void motorUpdate(){
  for(int i = 0; i < numMotors; i++){
    motors[i].update();
    motorStatusMsg.motor_id = i;
    motorStatusMsg.status = motors[i].isAlive() ? 1 : 0;
    motorStatusMsg.rpm = motors[i].rpm();
    motorStatusMsg.voltage = motors[i].voltage();
    motorStatusMsg.current = motors[i].current();
    motorStatusMsg.temperature = motors[i].temperature();
    pubMotorStatus.publish(&motorStatusMsg); 
  }
}

//This function converts a value between -1 and 1 to a throttle value
//Actual throttle values are -32767 to 32767
int16_t percentToThrottle(float t){
  return (int16_t)(t * 32767) * maxThrust;
}

void sensorUpdate(){
  int hydrophone_sensor_pin1 = analogRead(hydroPin1);
  int hydrophone_sensor_pin2 = analogRead(hydroPin2);
  int hydrophone_sensor_pin3 = analogRead(hydroPin3);

  //Calculate hydrophone heading and distance
  hydroMsg.distance = 0; 
  hydroMsg.degree = 0;
  pubHydro.publish(&hydroMsg);

  depthMsg.data = analogRead(depthPin);
  pubDepth.publish(&depthMsg);
}

void loop() {
  motorUpdate();
  sensorUpdate();
  nh.spinOnce();
  delay(1000); // Update at roughly 4 hz
}
