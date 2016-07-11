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
 * This arduino relays motor commands from ROS to the motors.
 * This script still contains code for reading motor status info
 * using BlueRobotics I2C library. However, none of it is in use, since
 * we switched to using PWM due to most of our BlueESCs blowing up.
*/

const int numMotors = 6;

ros::NodeHandle nh;
msurobosub::MotorStatus motorStatusMsg;
ros::Publisher pubMotorStatus("sensors/motor_status", &motorStatusMsg);

/*
Arduino_I2C_ESC motors[numMotors] = {
  Arduino_I2C_ESC(0x30),//ForwardPort
  Arduino_I2C_ESC(0x2F),//ForwardStar
  Arduino_I2C_ESC(0x35),//DepthFore
  Arduino_I2C_ESC(0x37),//DepthAft
  Arduino_I2C_ESC(0x38),//StrafeForward
  Arduino_I2C_ESC(0x36)//StrafeBack
};
*/

//Motor pins
int motorsPins[numMotors] = {
  0,//ForwardPort
  1,//ForwardStarboard
  2,//DepthFore
  3,//DepthAft
  4,//StrafeForward
  5//StrafeBack
};

Servo motors[numMotors];

//Array for direction motor runs
int direction[numMotors] = {-1, 1, 1, 1, -1, 1};
float lastMotorCommand[numMotors] = {0, 0, 0, 0, 0, 0};
int motorStatusFreq = 4;//How often we send motor updates in hertz
int lastMotor = 0;//Which motor did we send an update for last?
unsigned long motorUpdateTime = 0;//Time at which we should send next motor update

//Percent value indicating what power level we consider to be max thrust
//All received thrust commands will be scaled by this amount
float maxThrust = .75;

void motorCommandCallback(const msurobosub::MotorCommand& command){
  lastMotorCommand[0] = percentToThrottle(command.power[0]) * direction[0];
  lastMotorCommand[1] = percentToThrottle(command.power[1]) * direction[1];
  lastMotorCommand[2] = percentToThrottle(command.power[2]) * direction[2];
  lastMotorCommand[3] = percentToThrottle(command.power[3]) * direction[3];
  lastMotorCommand[4] = percentToThrottle(command.power[4]) * direction[4];
  lastMotorCommand[5] = percentToThrottle(command.power[5]) * direction[5];
}

ros::Subscriber<msurobosub::MotorCommand> subMotorCommand("command/motor", &motorCommandCallback);

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

	//PWM stuff
	motors[i].writeMicroseconds(lastMotorCommand[i]);
	

	//I2C stuff
	/*
    motors[i].update();
    motors[i].set(lastMotorCommand[i]);
	
	
    if(lastMotor == i && millis() > motorUpdateTime){
      motorStatusMsg.header = getHeader(motorStatusMsg.header);
      motorStatusMsg.motor_id = i;
      motorStatusMsg.connected = motors[i].isAlive() ? 1 : 0;
      motorStatusMsg.rpm = motors[i].rpm();
      motorStatusMsg.voltage = motors[i].voltage();
      motorStatusMsg.current = motors[i].current();
      motorStatusMsg.temperature = motors[i].temperature();
      pubMotorStatus.publish(&motorStatusMsg); 

      lastMotor = lastMotor == 5 ? 0 : ++lastMotor;
      motorUpdateTime = millis() + (1/motorStatusFreq*1000)/numMotors;
    }
	*/
  }
}

//This function converts a value between -1 and 1 to a throttle value
//I2C throttle values are -32767 to 32767
//PWM throttle values are 1100 to 1900, 1500 being 0
int16_t percentToThrottle(float t){
  t = constrain(t, -1, 1);
  #return (int16_t)(t * 32767) * maxThrust;//I2C
  return (int16_t)(1500 + t*400) * maxThrust;//PWM
}

void setup() {
  Wire.begin();
  nh.initNode();
  nh.advertise(pubMotorStatus);
  nh.subscribe(subMotorCommand);

  for(int i = 0; i < numMotors; i++){
    motors[i].attach(motorPin[i]);
    motors[i].writeMicroseconds(1500);
    delay(1000);
  }

  //Initialize messages
  motorStatusMsg.header = getHeader();
  motorStatusMsg.header.frame_id = "0";
}

void loop() {
  motorUpdate();
  nh.spinOnce();
}
