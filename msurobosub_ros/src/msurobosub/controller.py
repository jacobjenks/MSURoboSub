#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry 
from msurobosub.msg import MotorCommand
import math
import tf2_ros
import geometry_msgs.msg

msgOdom = None
msgOdomCommand = None
msgMot = None
msgOdomFront = None

pubMot = None


def odomCallback(msg):
	global msgOdom
	msgOdom = msg

def turnLeft(front, center, target): #Remember the ROS standard orientation
	if front.y == 0:	
		m = 1000000000
	else:
		m = front.x / front.y
	b = center.x

	if target.x > m*target.y + b:
		if front.y < 0:
			return false
		else:
			return true
	else:
		if front.y < 0:
			return true
		else:
			return false

def odomCommandCallback(msgOdomCommand):
	global msg
	msgOdomCommand = msg

def sendMotorCommand():
	global msgOdomCommand, msgOdom, msgMot, pubMot

	if msgOdom == None:
		return
		
	# Decide what motors to turn on, and send MotorCommand
	
	max_power = 1.00
	max_rotation = 0.10
	
	center = msgOdom.pose.pose.position
	target = msgOdomCommand.pose.pose.position
	try:
		trans = tfBuffer.lookup_transform('base_link', 'sub_front', rospy.Time())
	except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
		return

	front = trans.transform.translation
	
	x_comp = target.x - center.x
	y_comp = target.y - center.y
	z_comp = target.z - center.z
	
	unit_factor = math.sqrt(abs(x_comp**2) + abs(y_comp**2) + abs(z_comp**2))

	x_unit = x_comp / unit_factor
	z_unit = z_comp / unit_factor
	y_unit = y_comp / unit_factor

	unit_orientation_factor = math.sqrt(abs((front.x - center.x)**2) + abs((front.y - center.y)**2))

	x_orient = (front.x - center.x) / unit_orientation_factor
	y_orient = (front.y - center.y) / unit_orientation_factor

	#strafe thrusters
	if turnLeft(front, center, target):
		msgMot.power[4] = y_unit - max_rotation
		msgMot.power[5] = y_unit + max_rotation
	else:
		msgMot.power[4] = y_unit + max_rotation
		msgMot.power[5] = y_unit - max_rotation
	
	#forward thrusters
	if math.pi/4 >= math.acos((x_unit * x_orient) + (y_unit * y_orient)):
		msgMot.power[0] = x_unit
		msgMot.power[1] = x_unit

	#depth thrusters
	if z_unit > max_power:
		msgMot.power[2] = max_power
		msgMot.power[3] = max_power
	else:
		msgMot.power[2] = z_unit
		msgMot.power[3] = z_unit

	msgMot.motor_id = 0
	msgMot.power = 1
	pubMot.publish(msgMot)

def controller():
	global pubMot, msgMot
	pubMot = rospy.Publisher('command/motor', MotorCommand, queue_size=10)
	rospy.init_node('controller')

	rospy.Subscriber("odometry/filtered", Odometry, odomCallback)
	rospy.Subscriber("command/moveTo", Odometry, odomCommandCallback)

	msgMot = MotorCommand()
	msgMot.header.seq = 0
	msgMot.header.frame_id = "base_frame"

	rate = rospy.Rate(50)
	while not rospy.is_shutdown():
		sendMotorCommand()
		rate.sleep()

if __name__ == '__main__':
	try:
		controller()
	except rospy.ROSInterruptException:
		pass
