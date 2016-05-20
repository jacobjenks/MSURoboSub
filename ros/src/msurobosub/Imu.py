#!/usr/bin/env python
import serial
import rospy
from msurobosub.msg import Imu

imuPort = '/dev/ttyUSB0'
imuBaud = 115200

def imuTalker():
	pub = rospy.Publisher('imu', Imu, queue_size=10)
	rospy.init_node('imu')
	rate = rospy.Rate(100)#Update at GEDC-6E update rate
	imuMsg = Imu()
	
	ser = serial.Serial(imuPort, imuBaud)
	ser.flush()#make sure buffer is empty before we start looping

	while not rospy.is_shutdown():
		#get imu data
		data = ser.readline()
		if len(data) > 0:
			data = data.split(',')
			imuMsg.accel[0] = data[0] 
			imuMsg.accel[1] = data[1]
			imuMsg.accel[2] = data[2]
			imuMsg.gyro[0] = data[3]
			imuMsg.gyro[1] = data[4]
			imuMsg.gyro[2] = data[5]
			imuMsg.mag[0] = data[6]
			imuMsg.mag[1] = data[7]
			imuMsg.mag[2] = data[8]
			imuMsg.temp = data[9]
			pub.publish(imuMsg)
		rate.sleep()


if __name__ == '__main__':
	try:
		imuTalker()
	except rospy.ROSInterruptException:
		pass
