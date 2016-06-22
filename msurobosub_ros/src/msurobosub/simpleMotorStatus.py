#!/usr/bin/env python
import rospy
from msurobosub.msg import MotorStatus

msgMotor = None

def motorCB(msg):
	global msgMotor
	msgMotor[msg.motor_id] = msg
	
	
def simpleMotorStatus():
	global msgMotor

	rospy.init_node('simpleMotorStatus')
	rospy.Subscriber("sensors/motor_status", MotorStatus, motorCB)

	msgMotor = [None] * 6
	for i in range(0,6):
		msgMotor[i] = MotorStatus()
		msgMotor[i].header.seq = 0
		msgMotor[i].header.frame_id = ""

	rate = rospy.Rate(4)

	while not rospy.is_shutdown():
		print("Motor | Connected |  RPM  | Voltage | Current |   Temp")
		for i in range(0, 6):
			print("{:^5} | {:^9} | {:^5} | {:^7} | {:^7} | {:^8}").format(msgMotor[i].motor_id, msgMotor[i].connected, msgMotor[i].rpm, msgMotor[i].voltage, msgMotor[i].current, msgMotor[i].temperature)
		print("______________________________________________________")
		rate.sleep()

if __name__ == '__main__':
	try:
		simpleMotorStatus()
	except rospy.ROSInterruptException:
		pass
