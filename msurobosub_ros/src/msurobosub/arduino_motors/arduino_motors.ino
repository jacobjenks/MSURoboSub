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
/***** Pneumatics *****/
const int armUpDownPin = 33;
const int handOpenClosePin = 35;
const int dropLeftPin = 37;
const int dropRightPin = 39;
const int torpLeftPin = 41;
const int torpRightPin = 43;

/***** Sensors *****/
const int depthPin = 0;
//const int handPressure1 = 1;
//const int handPressure2 = 2;


bool pneumaticLock = true;
bool armDown = false;
bool handOpen = false;
float surfacePSI = 10.44;

int pneumaticShutoffPin = 0;//Pin to shut off
unsigned long pneumaticShutoffTime = 0;//Time at which we should turn off pneumatic valve

const int numMotors = 8;

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
	2, //Port Forward
	3, //Starboard Forward
	4, //Fore Strafe
	5, //Aft Strafe
	6, //Fore Port Depth
	7, //Fore Starboard Depth
	8, //Aft Port Depth
	9  //Aft Starboard Depth
};

Servo motors[numMotors];

//Array for direction motor runs
//float direction[numMotors] = {.96, 1, -.88, -1, -1, -1};
float direction[numMotors] = {1, 1, 1, 1, 1, 1, 1, 1}; //Find the direction here
int lastMotorCommand[numMotors] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
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
	lastMotorCommand[6] = percentToThrottle(command.power[6], 6);
	lastMotorCommand[7] = percentToThrottle(command.power[7], 7);
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
    case 41://Left Torpedo -> 1
      if(!pneumaticLock){
        nh.loginfo("Firing left torpedo");
        activatePneumatics(torpLeftPin, 250); //250?? Duration in Milliseconds?
      }
      break;
    case 43://Right Torpedo -> 2
      if(!pneumaticLock){
        nh.loginfo("Firing right torpedo");
        activatePneumatics(torpRightPin, 250);
      }
      break;
    case 37://Left Dropper -> 3
      if(!pneumaticLock){
        nh.loginfo("Releasing left dropper");
        activatePneumatics(dropLeftPin, 250);
      }
      break;
    case 39://Right Dropper -> 4
      if(!pneumaticLock){
        nh.loginfo("Releasing right dropper");
        activatePneumatics(dropRightPin, 250);
      }
      break;
    case 33://Arm Up/Down -> 5
      if(!pneumaticLock){
		if(!armDown) {
        	nh.loginfo("Lowering Arm");
			armDown = true;
        	activatePneumatics(armUpDownPin, 3000);//3000? 3000 Milliseconds?
		}
		else {
			nh.loginfo("Raising Arm");
			armDown = false;
			activatePneumatics(armUpDownPin, 3000);
		}
      }
      break;
    case 35://Hand Open/Close -> 6
      if(!pneumaticLock){
        if(!handOpen) {
			nh.loginfo("Opening Hand");
			handOpen = true;
        	activatePneumatics(handOpenClosePin, 3000);
		}
		else {
			nh.loginfo("Closing Hand");
			handOpen = false;
			activatePneumatics(handOpenClosePin, 3000);
		}
      }
      break;
    default:
      nh.logerror("Invalid pneumatics command");
  }
}

ros::Subscriber<msurobosub::MotorCommand> subMotorCommand("command/motor", &motorCommandCallback);
ros::Subscriber<msurobosub::PneumaticCommand> subPneumaticCommand("command/pneumatic", &pneumaticCommandCallback);

void activatePneumatics(int pin, int duration){
  if(pin == 33 && armDown) {
	digitalWrite(pin, LOW);
  }
  else if(pin == 35 && handOpen) {
	  digitalWrite(pin, LOW);
  }
  else {
    digitalWrite(pin, HIGH);
  }
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

      lastMotor = lastMotor == 7 ? 0 : ++lastMotor; //lastMotor == 5 ? 0 ->WTF?
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

  //Initialize pneumatics -> LOW not HIGH
  pinMode(torpLeftPin, OUTPUT);
  pinMode(torpRightPin, OUTPUT);
  pinMode(dropLeftPin, OUTPUT);
  pinMode(dropRightPin, OUTPUT);
  pinMode(armUpDownPin, OUTPUT);
  pinMode(handOpenClosePin, OUTPUT);
  digitalWrite(armUpDownPin, LOW);
  digitalWrite(handOpenClosePin, LOW);
  digitalWrite(torpLeftPin, LOW);
  digitalWrite(torpRightPin, LOW);
  digitalWrite(dropLeftPin, LOW);
  digitalWrite(dropRightPin, LOW);

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
