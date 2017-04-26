#!/usr/bin/env python
import rospy
from datetime import datetime
import math
from keyboard.msg import Key
from msurobosub.msg import MotorCommand
from msurobosub.msg import PneumaticCommand
from std_msgs.msg import Bool

msgMotor = None
msgPneu = None
msgMission = None

pubMotor = None
pubPneu = None
pubMission = None

commandTimeout = None
commandTimeoutDelay = 200

motorPower = 1
motorStopped = False

#Map motor activations to key presses
#Key id maps to list of motor ids to activate (keys), and the direction with which to activate (values)
keyMapping = {119: {0: 1, 1: 1},#w, forward
			  97:  {2: -1, 3: -1},#a, strafe left
			  115: {0: -1, 1: -1},#s, reverse
			  100: {2: 1, 3: 1},#d, strafe right
			  113: {2: -1, 3: 1},#q, rotate left
			  101: {2: 1, 3: -1},#e, rotate right
			  99:  {4: -1, 5: -1, 6: -1, 7: -1},#c, descend -> NOT sure about the -1 setting. Figure this out from live testing
			  32:  {4: 1, 5: 1, 6: 1, 7: 1},#space, ascend
			  120: {0: 0, 1: 0, 2: 0, 3: 0, 4: 0, 5: 0, 6: 0, 7: 0},#x, stop all motors 
			  116: {0: 1, 1: 1, 2: 1, 3: 1, 4: 1, 5: 1, 6: 1, 7: 1}#t, test all motors
			 }

#@param Key key: The message for the key that was pressed
#@param bool down: Whether the keypress was up or down
def userInput(key, down):
	global msgMotor, msgPneu, motorPower, keyMapping, commandTimeout, commandTimeoutDelay
	
	key = int(key.code)

	if not down:
		commandTimeout[key] = datetime.now().microsecond + commandTimeoutDelay
		return

	setMotorPower(key)
	
	#pneumatics
	if(key == 108 and down):#l, toggle pneumatic lock
		msgPneu.command = 0
	elif(key == 49 and down):#1, fire torpedo
		msgPneu.command = 1
	elif(key == 50 and down):#2, fire torpedo
		msgPneu.command = 2
	elif(key == 51 and down):#3, dropper
		msgPneu.command = 3
	elif(key == 52 and down):#4, dropper
		msgPneu.command = 4
	elif(key == 53 and down):#5, arm lower
		msgPneu.command = 5
	elif(key == 54 and down):#6, arm raise
		msgPneu.command = 6
	elif(key == 55 and down):#7 hand open
		msgPneu.command = 7
	elif(key == 56 and down):#8 hand close
		msgPneu.command = 8
	#other
	elif(key == 112 and down):#p, toggle mission pause
		msgMission.data = not msgMission.data
	

#Convert keypress to motor power setting, with optional power setting
def setMotorPower(key, power = None):
	global msgMotor, keyMapping, motorPower
	if power == None:
		power = motorPower
	if key in keyMapping:
		for key, value in keyMapping[key].items():
			msgMotor.power[key] =  value * power

'''
Motor IDs:
0 ForwardPort
1 ForwardStar
2 DepthForePort
3 DepthAftPort
4 DepthForeStar
5 DepthAftStar
6 StrafeTop
7 StrafeBottom
'''
def sendMotorCommand():
	global pubMotor, msgMotor, motorStopped
	msgMotor.header.seq += 1
	msgMotor.header.stamp = rospy.get_rostime()

#	if any(msg_mot is not 0 for msg_mot in msgMotor.power) or motorStopped is False:
	pubMotor.publish(msgMotor)

'''
Pneumatic IDs:
0 Pneumatic lock
1 Torpedo 1
2 Torpedo 2
3 Dropper 1
4 Dropper 2
5 Arm up/down 
6 Arm open/close
'''
def sendPneuCommand():
	global pubPneu, msgPneu
	if msgPneu.command != -1:
		msgPneu.header.seq += 1
		msgPneu.header.stamp = rospy.get_rostime()
		pubPneu.publish(msgPneu)
		msgPneu.command = -1

def sendMissionCommand():
	global pubMission, msgMission
	pubMission.publish(msgMission)

def keyDown(key):
	userInput(key, True)

def keyUp(key):
	userInput(key, False)

def main():
	global pubMotor, pubPneu, pubMission, msgMotor, msgPneu, msgMission, motorPower, commandTimeout

	commandTimeout = dict()

	rospy.init_node('UserInput')
	rospy.Subscriber("keyboard/keydown", Key, keyDown)
	rospy.Subscriber("keyboard/keyup", Key, keyUp)
	pubMotor = rospy.Publisher("command/motor", MotorCommand, queue_size=30)
	pubPneu = rospy.Publisher("command/pneumatic", PneumaticCommand, queue_size=30)
	pubMission = rospy.Publisher("command/missionToggle", Bool, queue_size=5)

	msgMotor = MotorCommand()
	msgMotor.header.seq = 0
	msgMotor.header.stamp = rospy.get_rostime()
	msgMotor.header.frame_id = "0"

	msgPneu = PneumaticCommand()
	msgPneu.header.seq = 0
	msgPneu.header.stamp = rospy.get_rostime()
	msgPneu.header.frame_id = "0"

	msgMission = Bool()
	msgMission.data = False

	rate = rospy.Rate(8)
	while not rospy.is_shutdown():
		#Keypresses time out after an interval to make keyboard control still work when connecting via remote desktop
		#since remote desktop constantly sends keyups/downs while a button is held down
		for key, value in commandTimeout.items():
			if datetime.now().microsecond > value:
				setMotorPower(key, 0)
				commandTimeout.pop(key, None)
			
		sendMotorCommand()
		sendPneuCommand()
		sendMissionCommand()
		rate.sleep()

	rospy.spin()


if __name__ == '__main__':
	main()
