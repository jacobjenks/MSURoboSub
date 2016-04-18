#!/usr/bin/env python

import rospy
import rosserial_python

def handleSendData(req):
	#Receive data from ROS and add to next message for Arduino

def handleReceiveData(req):
	#Receive data from Arduino and forward on to ROS
	
def ArduinoInterfaceServer():
	rospy.init_node('ArduinoInterface')
	s1 = rospy.Service('arduinoSendData', ArduinoInterface, handleSendData)
	s2 = rospy.Service('arduinoReceiveData', ArduinoInterface, handleReceiveData)
	print "Ready to interface with Arduino."
	rospy.spin()

if __name__ = "__main__":
	ArduinoInterfaceServer()