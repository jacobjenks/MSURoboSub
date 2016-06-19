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
const int torp1Pin = 4;
const int torp2Pin = 5;
const int drop1Pin = 2;
const int drop2Pin = 3;
const int armOpenPin = 7;
const int armClosePin = 6;
const int hydroPin1 = 0;
const int hydroPin2 = 1;
const int hydroPin3 = 2;
const int depthPin = 3;

bool pneumaticLock = true;

//PSI at water surface - change when elevation changes
//float surfacePSI = 10.696;
float surfacePSI = -1;

ros::NodeHandle nh;
msurobosub::MotorStatus motorStatusMsg;
ros::Publisher pubMotorStatus("sensors/motor_status", &motorStatusMsg);
msurobosub::Depth depthMsg;
ros::Publisher pubDepth("sensors/depth", &depthMsg);
msurobosub::Hydro hydroMsg;
ros::Publisher pubHydro("sensors/hydrophone", &hydroMsg);

Arduino_I2C_ESC motors[numMotors] = {
  Arduino_I2C_ESC(0x2F),//ForwardPort
  Arduino_I2C_ESC(0x30),//ForwardStar
  Arduino_I2C_ESC(0x35),//DepthFore
  Arduino_I2C_ESC(0x37),//DepthAft
  Arduino_I2C_ESC(0x36),//StrafeTop
  Arduino_I2C_ESC(0x38)//StrafeBottom
};

//Array for direction motor runs
int direction[numMotors] = {1, -1, 1, 1, 1, 1};
float lastMotorCommand[numMotors] = {0, 0, 0, 0, 0, 0};
int motorStatusFreq = 4;//How often we send motor updates in hertz
int lastMotor = 0;//Which motor did we send an update for last?
unsigned long motorUpdateTime = 0;//Time at which we should send next motor update

int pneumaticShutoffPin = 0;//Pin to shut off
unsigned long pneumaticShutoffTime = 0;//Time at which we should turn off pneumatic valve

float maxThrust = .75;//Percent value indicating what power level we consider to be max thrust

void motorCommandCallback(const msurobosub::MotorCommand& command){
  lastMotorCommand[0] = percentToThrottle(command.power[0]) * direction[0];
  lastMotorCommand[1] = percentToThrottle(command.power[0]) * direction[1];
  lastMotorCommand[2] = percentToThrottle(command.power[0]) * direction[2];
  lastMotorCommand[3] = percentToThrottle(command.power[0]) * direction[3];
  lastMotorCommand[4] = percentToThrottle(command.power[0]) * direction[4];
  lastMotorCommand[5] = percentToThrottle(command.power[0]) * direction[5];
}

void pneumaticCommandCallback(const msurobosub::PneumaticCommand& command){
  switch(command.command){
    case 0://Pneumatic lock
      if(pneumaticLock){
        pneumaticLock = false;
        nh.loginfo("Pneumatics unlocked");
      }
      else{
        pneumaticLock = true;
        nh.loginfo("Pneumatics locked");
      }
      break;
    case 1://Torp 1
      if(!pneumaticLock){
        nh.loginfo("Firing torpedo 1");
        activatePneumatics(torp1Pin, 250);
      }
      break;
    case 2://Torp 2
      if(!pneumaticLock){
        nh.loginfo("Firing torpedo 2");
        activatePneumatics(torp2Pin, 250);
      }
      break;
    case 3://Drop 1
      if(!pneumaticLock){
        nh.loginfo("Releasing dropper 1");
        activatePneumatics(drop1Pin, 250);
      }
      break;
    case 4://Drop 2
      if(!pneumaticLock){
        nh.loginfo("Releasing dropper 1");
        activatePneumatics(drop2Pin, 250);
      }
      break;
    case 5://Arm open
      if(!pneumaticLock){
        nh.loginfo("Opening arm");
        activatePneumatics(armOpenPin, 3000);
      }
      break;
    case 6://Arm close
      if(!pneumaticLock){
        nh.loginfo("Closing arm");
        activatePneumatics(armClosePin, 3000);
      }
      break;
    default:
      nh.logerror("Invalid pneumatics command");
  }
}

ros::Subscriber<msurobosub::MotorCommand> subMotorCommand("command/motor", &motorCommandCallback);
ros::Subscriber<msurobosub::PneumaticCommand> subPneumaticCommand("command/pneumatic", &pneumaticCommandCallback);

void activatePneumatics(int pin, int duration){
  digitalWrite(pin, LOW);
  pneumaticShutoffTime = millis() + duration;
  pneumaticShutoffPin = pin;
}

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
  updated.frame_id = h.frame_id;
  return updated;
}

//Update motor readings/power setting
void motorUpdate(){
  for(int i = 0; i < numMotors; i++){
    motors[i].update();
    motors[i].set(lastMotorCommand[i]);

    if(lastMotorUpdate == i && millis() > motorUpdateTime){
      motorStatusMsg.header = getHeader(motorStatusMsg.header);
      motorStatusMsg.motor_id = i;
      motorStatusMsg.connected = motors[i].isAlive() ? 1 : 0;
      motorStatusMsg.rpm = motors[i].rpm();
      motorStatusMsg.voltage = motors[i].voltage();
      motorStatusMsg.current = motors[i].current();
      motorStatusMsg.temperature = motors[i].temperature();
      pubMotorStatus.publish(&motorStatusMsg); 

      lastMotorUpdate = lastMotorUpdate == 5 ? 0 : lastMotorUpdate++;
      motorUpdateTime = millis() + (1/motorStatusFreq*1000)/numMotors;
    }
  }
}

//This function converts a value between -1 and 1 to a throttle value
//Actual throttle values are -32767 to 32767
int16_t percentToThrottle(float t){
  t = constrain(t, -1, 1);
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

  depthMsg.header = getHeader(depthMsg.header);
  //Convert depth sensor reading to PSI
  depthMsg.psi = (analogRead(depthPin) * .0048828125 - 1)*12.5;
  
  if(surfacePSI == -1)
    surfacePSI = depthMsg.psi;

  //Convert difference between surface and current PSI to depth in meters.
  depthMsg.depth = ((analogRead(depthPin) * .0048828125 - 1)*12.5 - surfacePSI)*.13197839577;
  pubDepth.publish(&depthMsg);
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
  motorStatusMsg.header.frame_id = "0";
  hydroMsg.header = getHeader();
  hydroMsg.header.frame_id = "hydro";
  depthMsg.header = getHeader();
  depthMsg.header.frame_id = "depth";
  
  // Optional: Add these two lines to slow I2C clock to 12.5kHz from 100 kHz
  // This is best for long wire lengths to minimize errors
  TWBR = 158;  
  TWSR |= bit (TWPS0);
}

void loop() {
  motorUpdate();
  sensorUpdate();

  //check for pneumatics shutoff
  if(pneumaticShutoffTime != 0 && millis() > pneumaticShutoffTime){
    digitalWrite(pneumaticShutoffPin, HIGH);
    pneumaticShutoffTime = 0;
  }
  
  nh.spinOnce();
}
