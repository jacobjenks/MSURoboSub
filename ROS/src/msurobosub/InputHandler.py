#!/usr/bin/env python

import rospy
import keyboard

def InputHandler():
	pub = rospy.Publisher('input', String, queue_size=10)#input is topic we publish to
	rospy.init_node('InputHandler', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		#listen for key presses and send them where they need to go
		
		#rospy.loginfo(hello_str)
		#pub.publish(hello_str)
		rate.sleep()

if __name__ = "__main__":
	try:
		InputHandler()
	except rospy.ROSInterruptException:
		pass