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
 * This arduino relays pneumatic commands from ROS to the hardware, and relays
 * depth/hydrophone sensor data to ROS.
 * 
 * A rosserial_python node (started in the launch file) listens to the serial channel, and
 * relays all of this information between the Arduino and ROS
*/

const int numMotors = 8; //Unused Declaration

/****** Pneumatics *******/
const int armUpDownPin = 33;
const int handOpenClosePin = 35;
const int dropLeftPin = 37;
const int dropRightPin = 39;
const int torpLeftPin = 41;
const int torpRightPin = 43;

/****** Sensors *******/
const int depthPin = A0;
const int handPressure1 = A1;
const int handPressure2 = A2;
//const int hydroPin1;
//const int hydroPin2;
//const int hydroPin3;

bool pneumaticLock = true;
bool armDown = false;
bool handOpen = false;

//PSI at water surface - change when elevation changes
//float surfacePSI = 10.696;
float surfacePSI = -1;

ros::NodeHandle nh;
msurobosub::Depth depthMsg;
ros::Publisher pubDepth("sensors/depth", &depthMsg);
msurobosub::Hydro hydroMsg;
ros::Publisher pubHydro("sensors/hydrophone", &hydroMsg);

int pneumaticShutoffPin = 0;//Pin to shut off
unsigned long pneumaticShutoffTime = 0;//Time at which we should turn off pneumatic valve

//I have no idea what sthese values should be - ask the EE's or the inter-highway
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
    case 41://Left Torpedo
      if(!pneumaticLock){
        nh.loginfo("Firing torpedo 1");
        activatePneumatics(torpLeftPin, 250);
      }
      break;
    case 43://Right Torpedo
      if(!pneumaticLock){
        nh.loginfo("Firing torpedo 2");
        activatePneumatics(torpRightPin, 250);
      }
      break;
    case 37://Left Dropper
      if(!pneumaticLock){
        nh.loginfo("Releasing dropper 1");
        activatePneumatics(dropLeftPin, 250);
      }
      break;
    case 39://Right Dropper
      if(!pneumaticLock){
        nh.loginfo("Releasing dropper 1");
        activatePneumatics(dropRightPin, 250);
      }
      break;
    case 33://Arm Up/Down
      if(!pneumaticLock){
		if(!armDown) {
        	nh.loginfo("Lowering arm");
        	activatePneumatics(armUpDownPin, 3000); //No idea what these should actually be
		}
		else {
			nh.loginfo("Raising arm");
			activatingPneumatics(armUpDownPin, 3000); //No Idea what these should actually be
		}
      }
      break;
    case 35://Arm close
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

void sensorUpdate(){
  /* Hydrophone Sensors
  int hydrophone_sensor_pin1 = analogRead(hydroPin1);
  int hydrophone_sensor_pin2 = analogRead(hydroPin2);
  int hydrophone_sensor_pin3 = analogRead(hydroPin3);

  //Calculate hydrophone heading and distance
  hydroMsg.header = getHeader(hydroMsg.header);
  hydroMsg.distance = 0; 
  hydroMsg.degree = 0;
  pubHydro.publish(&hydroMsg);
  */

  //Depth sensors
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
  //nh.advertise(pubHydro);
  nh.advertise(pubDepth);
  nh.subscribe(subPneumaticCommand);

  //Initialize pneumatics
  pinMode(torpLeftPin, OUTPUT);
  pinMode(torpRightPin, OUTPUT);
  pinMode(dropLeftPin, OUTPUT);
  pinMode(dropRightPin, OUTPUT);
  pinMode(armUpDownPin, OUTPUT);
  pinMode(handOpenClosePin, OUTPUT);
  digitalWrite(armUpDownPin, HIGH);
  digitalWrite(handOpenClosePin, HIGH);
  digitalWrite(torpLeftPin, HIGH);
  digitalWrite(torpRightPin, HIGH);
  digitalWrite(dropLeftPin, HIGH);
  digitalWrite(dropRightPin, HIGH);

  //Initialize messages
  //hydroMsg.header = getHeader();
  //hydroMsg.header.frame_id = "hydro";
  depthMsg.header = getHeader();
  depthMsg.header.frame_id = "depth";
  
  // Optional: Add these two lines to slow I2C clock to 12.5kHz from 100 kHz
  // This is best for long wire lengths to minimize errors
  TWBR = 158;  
  TWSR |= bit (TWPS0);
}

void loop() {
  sensorUpdate();

  //check for pneumatics shutoff
  if(pneumaticShutoffTime != 0 && millis() > pneumaticShutoffTime){
    digitalWrite(pneumaticShutoffPin, HIGH);
    pneumaticShutoffTime = 0;
  }

  nh.spinOnce();
}
