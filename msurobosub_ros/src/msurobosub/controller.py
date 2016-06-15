#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry 
from msurobosub.msg import MotorCommand

msgOdom = None
msgOdomCommand = None
msgMot = None

pubMot = None


def odomCallback(msg):
	global msgOdom
	msgOdom = msg


def odomCommandCallback(msg):
	global msgOdomCommand, msgMot, pubMot
	msgOdomCommand = msgOdomCommand

	# Decide what motors to turn on, and send MotorCommand
	pid()

	msgMot.motor_id = 0
	msgMot.power = 1
	pubMot.publish(msgMot)

#Make sure motor command is between -1 and 1
def clampMotorCommand(msgMot):
	if math.abs(msgMot.power) > 1:
		msgMot.power = math.copysign(1, msgMot.power)
	return msgMot

def pid():
	print("PID")

def controller():
	global pubMot, msgMot
	pubMot = rospy.Publisher('command/motor', MotorCommand, queue_size=10)
	rospy.init_node('controller')

	rospy.Subscriber("odometry/filtered", Odometry, odomCallback)
	rospy.Subscriber("command/conOdom", Odometry, odomCommandCallback)

	msgMot = MotorCommand()
	msgMot.header.seq = 0
	msgMot.header.frame_id = "base_frame"

	rospy.spin()

if __name__ == '__main__':
	try:
		conroller()
	except rospy.ROSInterruptException:
		pass
