#!/usr/bin/env python

import rospy
from msurobosub.msg import MotorCommand, VisualCoordinates
import math
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry

coordinates = None # Where we need to go
"""
0: Port Forward
1: Starboadrd Forward
2: Fore Strafe
3: Aft Strafe
4: Fore port Depth
5: Fore Starboard Depth
6: Aft Port Depth
7: Aft Starboard Depth
"""
motors = None # MotorCommand msg
motor_publisher = None # publisher for motor_message

max_power = 1.00
max_rotation = 0.10 # Is this important to me?

def send_motor_command():
	global motors, motor_publisher, coordinates
	
	if coordinates == None: # Shut off and return
		for i in range(len(motors)):
			motors[i].power = 0
		motor_publisher.publish(motors)
		return
	
	motors.power[0] = max_power if coordinates.z > 2 else (max_power/4)
	motors.power[1] = max_power if coordinates.z > 2 else (max_power/4)
	
	motors.power[2] = coordinates.x if abs(coordinates.x) <= max_power else max_power
	motors.power[3] = coordinates.x if abs(coordinates.x) <= max_power else max_power 
	
	motors.power[4] = coordinates.y if abs(coordinates.y) <= max_power else max_power
	motors.power[5] = coordinates.y if abs(coordinates.y) <= max_power else max_power
	motors.power[6] = coordinates.y if abs(coordinates.y) <= max_power else max_power
	motors.power[7] = coordinates.y if abs(coordinates.y) <= max_power else max_power
		
	motor_publisher.publish(motors)

def visual_controller_callback(visual_coordinates):
	global coordinates
	coordinates = visual_coordinates

def visual_controller():
	global motor_publisher, motors
	motor_publisher = rospy.Publisher('command/motor', MotorCommand, queue_size=16)
	rospy.init_node('visual_controller')

	# rospy.Subscriber('command/moveTo', Odometry, odom_command_callback)
	"""
	object_detector.py publish 'visual/coordinates' -> task.py 
	
	task.py publish 'visual/moveTo' -> visual_controller.py
	"""
	rospy.Subscriber('visual/moveTo', VisualCoordinates, visual_controller_callback)

	motors = MotorCommand()
	motors.header.seq = 0
	motors.header.frame_id = "base_link" # Need to update for new robosub - if even important for visual controller

	rate = rospy.Rate(50)
	while not rospy.is_shutdown():
		send_motor_command()
		rate.sleep()

def test_visual_controller():
	pass

if __name__ == "__main__":
	try:
		visual_controller()
		# test_visual_controller() # Give some fake coordinates to an object
	except rospy.ROSInterruptException:
		pass
