#!/usr/bin/env python
#--------------------------------------------------------------------#
#KeyBoard SubDriver
#
#
#
#
#--------------------------------------------------------------------#

import rospy
from std_msgs.msg import String

def move_sub(data):
	if (#strafe right,  D pressed)
		#code
	elif (#strafe left, A pressed)
		#code
	elif (#forward, W pressed)
		#code
	elif (#backward, S pressed)
		#code
	elif (#turn right, E pressed)
		#code
	elif (#turn left, Q pressed)
		#code
	elif (#up, Z pressed)
		#code
	elif (#down, X pressed)
		#code

def key_listener():
	rospy.init_node('key_listener', anonymous=True)
	rospy.Subscriber("key_strokes", int, move_sub)
	rospy.spin()


if __name__ == '__main__':
	key_listener()
