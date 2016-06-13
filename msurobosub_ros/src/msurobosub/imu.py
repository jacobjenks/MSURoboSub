#!/usr/bin/env python
import serial
import rospy
import re
import numpy as np
from sensor_msgs.msg import Imu, Temperature
import math

imuPort = '/dev/ttyUSB0'
imuBaud = 115200
ser = None 

def imuTalker():
	global ser
	pubImu = rospy.Publisher('sensors/imu', Imu, queue_size=10)
	pubTemp = rospy.Publisher('sensors/temperature', Temperature, queue_size=10)
	rospy.init_node('imu')
	rate = rospy.Rate(100)#Update at GEDC-6E update rate

	imuMsg = Imu()
	imuMsg.header.seq = 0
	imuMsg.header.frame_id = "imu0"
	
	tempMsg = Temperature()
	tempMsg.header.seq = 0
	tempMsg.header.frame_id = "imu0"

	#These need to be updated to reflect actual covariance
	imuMsg.orientation_covariance = [1.0,0.0,0.0,
									 0.0,1.0,0.0,
									 0.0,0.0,1.0]

	imuMsg.angular_velocity_covariance = [1,0.0,0.0,
					  					0.0,1,0.0,
					  					0.0,0.0,1]

	imuMsg.linear_acceleration_covariance = [1,0.0,0.0,
						 					0.0,1,0.0,
						 					0.0,0.0,1]

	try: 
		ser = serial.Serial(imuPort, imuBaud)
	except serial.SerialException:
		rospy.logfatal("Error connecting to IMU.")


	ser.flush()#make sure buffer is empty before we start looping

	while not rospy.is_shutdown():
		imuMsg.header.stamp = rospy.get_rostime() 
		imuMsg.header.seq += 1

		#Quaternion mag data
		data = getImuData("$PSPA,QUAT\r\n")
		imuMsg.orientation.w = data[0]
		imuMsg.orientation.x = data[1]
		imuMsg.orientation.y = data[2]
		imuMsg.orientation.z = data[3]

		#Gyro data
		#Converted from  millidegrees to radians
		data = getImuData("$PSPA,G\r\n")
		imuMsg.angular_velocity.x = data[0] * math.pi/180/1000
		imuMsg.angular_velocity.y = data[1] * math.pi/180/1000
		imuMsg.angular_velocity.z = data[2] * math.pi/180/1000
		
		#Accelerometer data
		#Converted from milli-g's to m/s^2
		data = getImuData("$PSPA,A\r\n")
		imuMsg.linear_acceleration.x = data[0] * 9.80665/1000
		imuMsg.linear_acceleration.y = data[1] * 9.80665/1000
		imuMsg.linear_acceleration.z = data[2] * 9.80665/1000

		#Temperature data
		data = getImuData("$PSPA,Temp\r\n")
		tempMsg.header.stamp = rospy.get_rostime()
		tempMsg.header.seq += 1
		tempMsg.temperature = data[0]

		pubImu.publish(imuMsg)
		pubTemp.publish(tempMsg)

		rate.sleep()

def getImuData(command):
	global ser
	ser.write(command)
	data = ser.readline()
	values = np.array(re.findall('([-\d.]+)', data)).astype(np.float)
	return values


if __name__ == '__main__':
	try:
		imuTalker()
	except rospy.ROSInterruptException:
		pass
