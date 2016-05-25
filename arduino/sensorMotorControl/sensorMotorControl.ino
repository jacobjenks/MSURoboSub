#include <Wire.h>
#include "Arduino_I2C_ESC.h"
#include <ctype.h>
#include <ros.h>
#include <ros/time.h>
#include <msurobosub/MotorStatus.h>
#include <msurobosub/Hydro.h>
#include <msurobosub/Depth.h>
#include <msurobosub/MotorCommand.h>
#include <msurobosub/PneumaticCommand.h>
#include <std_msgs/Header.h>

/*
 * This arduino relays pneumatic/motor commands from ROS to the hardware, and relays
 * depth/hydrophone/motor sensor data to ROS.
 * 
 * A rosserial_python node (started in the launch file) listens to the serial channel, and
 * relays all of this information between the Arduino and ROS
*/

const int numMotors = 6;
const int torp1Pin = 2;
const int torp2Pin = 3;
const int drop1Pin = 4;
const int drop2Pin = 5;
const int armOpenPin = 6;
const int armClosePin = 7;
const int hydroPin1 = 0;
const int hydroPin2 = 1;
const int hydroPin3 = 2;
const int depthPin = 3;

//PSI at water surface - change when elevation changes
const float surfacePSI = 15;

ros::NodeHandle nh;
msurobosub::MotorStatus motorStatusMsg;
ros::Publisher pubMotorStatus("motorStatus", &motorStatusMsg);
msurobosub::Depth depthMsg;
ros::Publisher pubDepth("depth", &depthMsg);
msurobosub::Hydro hydroMsg;
ros::Publisher pubHydro("hydrophone", &hydroMsg);

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
  if(command.power <= 1 && command.power >= -1)
    motors[command.motor_id].set(percentToThrottle(command.power));
  else
    nh.logerror("Invalid motor command");
}

void pneumaticCommandCallback(const msurobosub::PneumaticCommand& command){
  switch(command.command){
    case 0://Torp 1
      activatePneumatics(torp1Pin, 250);
      break;
    case 1://Torp 2
      activatePneumatics(torp2Pin, 250);
      break;
    case 2://Drop 1
      activatePneumatics(drop1Pin, 250);
      break;
    case 3://Drop 2
      activatePneumatics(drop2Pin, 250);
      break;
    case 4://Arm open
      activatePneumatics(armOpenPin, 3000);
      break;
    case 5://Arm close
      activatePneumatics(armClosePin, 3000);
      break;
    default:
      nh.logerror("Invalid pneumatics command");
  }
}

void activatePneumatics(int pin, int duration){
  digitalWrite(pin, LOW);
  delay(duration);
  digitalWrite(pin, HIGH);
}

ros::Subscriber<msurobosub::MotorCommand> subMotorCommand("motorCommand", &motorCommandCallback);
ros::Subscriber<msurobosub::PneumaticCommand> subPneumaticCommand("pneumaticCommand", &pneumaticCommandCallback);

std_msgs::Header getHeader(){
  std_msgs::Header updated = std_msgs::Header();
  updated.seq = 0;
  updated.stamp = nh.now();
  updated.frame_id = "0";
  return updated;
}

std_msgs::Header getHeader(std_msgs::Header h){
  std_msgs::Header updated = std_msgs::Header();
  updated.seq = h.seq + 1;
  updated.stamp = nh.now();
  updated.frame_id = "0";
  return updated;
}

void setup() {
  Wire.begin();
  nh.initNode();
  nh.advertise(pubMotorStatus);
  nh.advertise(pubHydro);
  nh.advertise(pubDepth);
  nh.subscribe(subMotorCommand);
  nh.subscribe(subPneumaticCommand);

  //Initialize pneumatics
  pinMode(torp1Pin, OUTPUT);
  pinMode(torp2Pin, OUTPUT);
  pinMode(drop1Pin, OUTPUT);
  pinMode(drop2Pin, OUTPUT);
  pinMode(armOpenPin, OUTPUT);
  pinMode(armClosePin, OUTPUT);
  digitalWrite(armOpenPin, HIGH);
  digitalWrite(armClosePin, HIGH);
  digitalWrite(torp1Pin, HIGH);
  digitalWrite(torp2Pin, HIGH);
  digitalWrite(drop1Pin, HIGH);
  digitalWrite(drop2Pin, HIGH);

  //Initialize messages
  motorStatusMsg.header = getHeader();
  hydroMsg.header = getHeader();
  depthMsg.header = getHeader();
  
  // Optional: Add these two lines to slow I2C clock to 12.5kHz from 100 kHz
  // This is best for long wire lengths to minimize errors
  //TWBR = 158;  
  //TWSR |= bit (TWPS0);
}


//Run actual update function that updates values from motor controllers
void motorUpdate(){
  for(int i = 0; i < numMotors; i++){
    motors[i].update();
    motorStatusMsg.header = getHeader(motorStatusMsg.header);
    motorStatusMsg.motor_id = i;
    motorStatusMsg.connected = motors[i].isAlive() ? 1 : 0;
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
  hydroMsg.header = getHeader(hydroMsg.header);
  hydroMsg.distance = 0; 
  hydroMsg.degree = 0;
  pubHydro.publish(&hydroMsg);

  //Convert depth sensor reading to PSI
  depthMsg.header = getHeader(depthMsg.header);
  depthMsg.depth = ((analogRead(depthPin) * .0048828125 - 1)*12.5 - surfacePSI)/0.433;
  pubDepth.publish(&depthMsg);
}

void loop() {
  motorUpdate();
  sensorUpdate();
  nh.spinOnce();
  delay(1000); // Update at roughly 4 hz
}
