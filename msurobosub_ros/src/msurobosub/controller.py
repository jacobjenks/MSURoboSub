#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry 
from msurobosub.msg import MotorCommand
import math
import tf2_ros
import geometry_msgs.msg

'''
Rewite controller logic to integrate two additional thrusters
Double check linear algebra calculations
Are linear transformations the right tool to use here
Does this even matter if I am just going to be using the visual controller?
'''
msgOdom = None # This is where the sub is
msgOdomCommand = None # This is where we want to go
msgOdomFront = None # This is the front of the sub
msgMot = None # This is the MotorCommand msg 

pubMot = None # This is the publisher for msgMot

max_power = 1.00
max_rotation = 0.10

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
			return False
		else:
			return True
	else:
		if front.y < 0:
			return True
		else:
			return False

def odomCommandCallback(msgOdomCommand):
	global msg
	msgOdomCommand = msg

def sendMotorCommand():
	global msgOdomCommand, msgOdom, msgMot, pubMot, max_power, max_rotation

	if msgOdom == None:
		return
		
	# Decide what motors to turn on, and send MotorCommand
		
	center = msgOdom.pose.pose.position
	target = msgOdomCommand.pose.pose.position
	
	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)

	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		try:
			trans = tfBuffer.lookup_transform('base_link', 'sub_front',  rospy.Time())
			break
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        		rate.sleep()
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
	if turnLeft(front, center, target) and y_unit == 0:
		msgMot.power[2] = 0
		msgMot.power[3] = 0
	elif turnLeft(front, center, target):
		msgMot.power[2] = y_unit - max_rotation
		msgMot.power[3] = y_unit + max_rotation
	else:
		msgMot.power[2] = y_unit + max_rotation
		msgMot.power[3] = y_unit - max_rotation
	
	#forward thrusters
	if math.pi/4 >= math.acos((x_unit * x_orient) + (y_unit * y_orient)):
		msgMot.power[0] = x_unit
		msgMot.power[1] = x_unit

	#depth thrusters
	if z_unit > max_power:
		msgMot.power[4] = max_power
		msgMot.power[5] = max_power
		msgMot.power[6] = max_power
		msgMot.power[7] = max_power
	else:
		msgMot.power[4] = z_unit
		msgMot.power[5] = z_unit
		msgMot.power[6] = z_unit
		msgMot.power[7] = z_unit
	
	pubMot.publish(msgMot)

def visualThrustCB(objectMsg):
	global max_power, max_rotation

	centerThresh = .2
	camCenterX= objectMsg.camera_info.width/2
	camCenterY = objectMsg.camera_info.height/2

	objectCenterX = objectMsg.x + objectMsg.width/2
	objectCenterY = objectMsg.y + objectMsg.height/2
	
	# If target is already centered, move forward, and correct more slowly
	if camCenterX - (camCenterX * centerThresh) <= objectCenterX <= camCenterX + (camCenterX * centerThresh):
		msgMot.power[0] = max_power
		msgMot.power[1] = max_power
		translatePower = max_power/4
	else:
		translatePower = max_power/2

	
	if objectCenterX < camCenterX:
		msgMot.power[2] = translatePower
		msgMot.power[3] = translatePower
	else:
		msgMot.power[2] = translatePower * -1
		msgMot.power[3] = translatePower * -1

	if objectCenterY < camCenterY:
		msgMot.power[4] = translatePower * -1
		msgMot.power[5] = translatePower * -1
		msgMot.power[6] = translatePower * -1
		msgMot.power[7] = translatePower * -1
	else:
		msgMot.power[4] = translatePower
		msgMot.power[5] = translatePower
		msgMot.power[6] = translatePower
		msgMot.power[7] = translatePower

#function spin never called
def spin(left = True):
	global max_rotate

	rotate = max_rotate
	if not left:
		rotate *= -1	

	msgMot.power[2] = rotate
	msgMot.power[3] = rotate

def controller():
	global pubMot, msgMot
	pubMot = rospy.Publisher('command/motor', MotorCommand, queue_size=16)
	rospy.init_node('controller')

	rospy.Subscriber("odometry/filtered", Odometry, odomCallback)
	rospy.Subscriber("command/moveTo", Odometry, odomCommandCallback)

	msgMot = MotorCommand()
	msgMot.header.seq = 0
	msgMot.header.frame_id = "base_link"

	rate = rospy.Rate(50)
	while not rospy.is_shutdown():
		sendMotorCommand()
		rate.sleep()

def testController():
	global msgOdom, msgOdomCommand

	msgOdomCommand = Odometry()
	msgOdomCommand.pose.pose.position.x = -2
	msgOdomCommand.pose.pose.position.y = 2
	msgOdomCommand.pose.pose.position.z = 2
	
	msgOdom = Odometry()
	msgOdom.pose.pose.position.x = 0
	msgOdom.pose.pose.position.y = 0
	msgOdom.pose.pose.position.z = 0

if __name__ == '__main__':
	try:
		testController()
		#controller()
	except rospy.ROSInterruptException:
		pass
