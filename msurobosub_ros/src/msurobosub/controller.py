#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry 
from msurobosub.msg import MotorCommand

import math

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
	
	max_power = 1

	x_comp = msgOdomCommand.pose.pose.position.x - msgOdom.pose.pose.position.x
	y_comp = msgOdomCommand.pose.pose.position.y - msgOdom.pose.pose.position.y
	z_comp = msgOdomCommand.pose.pose.position.z - msgOdom.pose.pose.position.z

	unit_factor = math.sqrt(abs(x_comp**2) + abs(y_comp**2) + abs(z_comp**2))

	x_unit = x_comp / unit_factor
	y_unit = y_comp / unit_factor
	z_unit = z_comp / unit_factor

	if x_unit > 0:
		#turn 90 degrees to the right
		msgMot.power[4] = max_power
		msgMot.power[5] = -1 * max_power
	if x_unit > 0:
		msgMot.power[4] = x_unit
		msgMot.power[5] = x_unit * -1
	else:
		msgMot.power[4] = -1 * x_unit
		msgMot.power[5] = x_unit
	msgMot.power[0] = z_unit
	msgMot.power[1] = z_unit
	msgMot.power[2] = y_unit
	msgMot.power[3] = y_unit

	msgMot.motor_id = 0
	msgMot.power = 1
	pubMot.publish(msgMot)

#Make sure motor command is between -1 and 1
def clampMotorCommand(msgMot):
	if math.abs(msgMot.power) > 1:
		msgMot.power = math.copysign(1, msgMot.power)
	return msgMot

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
