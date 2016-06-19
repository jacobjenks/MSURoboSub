#!/usr/bin/env python
#--------------------------------------------------------------------#
#
#
#
#
#--------------------------------------------------------------------#

import rospy
from keyboard.msg import Key
from msurobosub.msg import MotorCommand
from msurobosub.msg import PneumaticCommand

comPneu = None
comMotor = None
msgMotor = None
msgPneu = None

motorPower = .01

#@param Key key: The message for the key that was pressed
#@param bool down: Whether the keypress was up or down
def userInput(key, down):
	global msgMotor, msgPneu, comPneu, comMotor, motorPower

	#turn motors on if keydown, turn off if keyup
	activate = motorPower if down else 0 

	#motors
	if(key.code == 119):#w, forward
		msgMotor.power[0] = 1 * activate
		msgMotor.power[1] = 1 * activate
	elif(key.code == 97):#a, strafe left
		msgMotor.power[4] = -1 * activate
		msgMotor.power[5] = -1 * activate
	elif(key.code == 115):#s, backward
		msgMotor.power[0] = -1 * activate
		msgMotor.power[1] = -1 * activate
	elif(key.code == 100):#d, strafe right
		msgMotor.power[4] = 1 * activate
		msgMotor.power[5] = 1 * activate
	elif(key.code == 113):#q, rotate left
		msgMotor.power[4] = -1 * activate
		msgMotor.power[5] = 1 * activate
	elif(key.code == 101):#e, rotate right
		msgMotor.power[4] = 1 * activate
		msgMotor.power[5] = -1 * activate
	elif(key.code == 99):#c, descend
		msgMotor.power[2] = -1 * activate
		msgMotor.power[3] = -1 * activate
	elif(key.code == 32):#space, ascend
		msgMotor.power[2] = 1 * activate
		msgMotor.power[3] = 1 * activate
	#pneumatics
	elif(key.code == 108 and down):#l, toggle pneumatic lock
		msgPneu.command = 0
	elif(key.code == 49 and down):#1, fire torpedo
		msgPneu.command = 1
	elif(key.code == 50 and down):#2, fire torpedo
		msgPneu.command = 2
	elif(key.code == 51 and down):#3, dropper
		msgPneu.command = 3
	elif(key.code == 52 and down):#4, dropper
		msgPneu.command = 4
	elif(key.code == 53 and down):#5, arm open
		msgPneu.command = 5
	elif(key.code == 54 and down):#6, arm close
		msgPneu.command = 6

'''
Motor IDs:
0 ForwardPort
1 ForwardStar
2 DepthFore
3 DepthAft
4 StrafeTop
5 StrafeBottom
'''
def sendMotorCommand():
	global pubMotor, msgMotor
	msgMotor.header.seq += 1
	msgMotor.header.stamp = rospy.get_rostime()
	pubMotor.publish(msgMotor)

'''
Pneumatic IDs:
0 Pneumatic lock
1 Torpedo 1
2 Torpedo 2
3 Dropper 1
4 Dropper 2
5 Arm open 
6 Arm close 
'''
def sendPneuCommand():
	global pubPneu, msgPneu
	if pneu.command != -1:
		msgPneu.header.seq += 1
		msgPneu.header.stamp = rospy.get_rostime()
		pubPneu.publish(msgPneu)
		pneu.command = -1

def keyDown(key):
	userInput(key, True)

def keyUp(key):
	userInput(key, False)

def main():
	global pubMotor, pubPneu, msgMotor, msgPneu, motorPower
	rospy.init_node('UserInput')
	rospy.Subscriber("keyboard/keydown", Key, keyDown)
	rospy.Subscriber("keyboard/keyup", Key, keyUp)
	pubMotor = rospy.Publisher("command/motor", MotorCommand, queue_size=30)
	pubPneu = rospy.Publisher("command/pneumatic", PneumaticCommand, queue_size=30)

	msgMotor = MotorCommand()
	msgMotor.header.seq = 0
	msgMotor.header.stamp = rospy.get_rostime()
	msgMotor.header.frame_id = "0"

	msgPneu = PneumaticCommand()
	msgPneu.header.seq = 0
	msgPneu.header.stamp = rospy.get_rostime()
	msgPneu.header.frame_id = "0"

	rate = rospy.Rate(4)
	while not rospy.is_shutdown():
		sendMotorCommand()
		sendPneuCommand()
		rate.sleep()

	rospy.spin()


if __name__ == '__main__':
	main()
