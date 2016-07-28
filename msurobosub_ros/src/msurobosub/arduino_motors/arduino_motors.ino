/*
 * This arduino relays motor commands from ROS to the motors.
 * This script still contains code for reading motor status info
 * using BlueRobotics I2C library. However, none of it is in use, since
 * we switched to using PWM due to most of our BlueESCs blowing up.
*/
#include <Wire.h>
#include <Servo.h>
#include <ctype.h>
#include <ros.h>
#include <ros/time.h>
#include <msurobosub/Hydro.h>
#include <msurobosub/Depth.h>
#include <msurobosub/MotorCommand.h>
#include <msurobosub/PneumaticCommand.h>
#include <std_msgs/Header.h>

const int torp1Pin = 2;
const int torp2Pin = 3;
const int drop1Pin = 4;
const int drop2Pin = 5;
const int armOpenPin = 6;
const int armClosePin = 7;
const int depthPin = 0;

bool pneumaticLock = true;
float surfacePSI = 10.44;

int pneumaticShutoffPin = 0;//Pin to shut off
unsigned long pneumaticShutoffTime = 0;//Time at which we should turn off pneumatic valve

const int numMotors = 6;

ros::NodeHandle nh;
msurobosub::Depth depthMsg;
ros::Publisher pubDepth("sensors/depth", &depthMsg);

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
int motorPins[numMotors] = {
  46,//ForwardPort
  42,//ForwardStarboard
  44,//DepthFore
  36,//DepthAft
  40,//StrafeForward
  38//StrafeBack
};

Servo motors[numMotors];

//Array for direction motor runs
float direction[numMotors] = {1, 1, -.88, -1, -1, -1};
int lastMotorCommand[numMotors] = {1500, 1500, 1500, 1500, 1500, 1500};
int lastMotor = 0;//Which motor did we send an update for last?
unsigned long motorUpdateTime = 0;//Time at which we should send next motor update

//Percent value indicating what power level we consider to be max thrust
//All received thrust commands will be scaled by this amount
float maxThrust = .75;

void motorCommandCallback(const msurobosub::MotorCommand& command){
  lastMotorCommand[0] = percentToThrottle(command.power[0], 0);
  lastMotorCommand[1] = percentToThrottle(command.power[1], 1);
  lastMotorCommand[2] = percentToThrottle(command.power[2], 2);
  lastMotorCommand[3] = percentToThrottle(command.power[3], 3);
  lastMotorCommand[4] = percentToThrottle(command.power[4], 4);
  lastMotorCommand[5] = percentToThrottle(command.power[5], 5);
}

void pneumaticCommandCallback(const msurobosub::PneumaticCommand& command){
  switch(command.command){
    case 0://Pneumatic lock
      if(pneumaticLock){
        pneumaticLock =  false;
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
int percentToThrottle(float t, int motor){
  t = constrain(t, -1, 1);
  //return (int16_t)(t * 32767) * maxThrust;//I2C
  return int(1500 + t*400*maxThrust*direction[motor]);//PWM
}

void sensorUpdate(){
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
  nh.subscribe(subMotorCommand);
  nh.advertise(pubDepth);
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
  depthMsg.header = getHeader();
  depthMsg.header.frame_id = "depth";

  for(int i = 0; i < numMotors; i++){
    motors[i].attach(motorPins[i]);
    motors[i].writeMicroseconds(1500);
    delay(1000);
  }
}

void loop() {
  sensorUpdate();

  //check for pneumatics shutoff
  if(pneumaticShutoffTime != 0 && millis() > pneumaticShutoffTime){
    digitalWrite(pneumaticShutoffPin, HIGH);
    pneumaticShutoffTime = 0;
  }
  
  motorUpdate();
  nh.spinOnce();
}
