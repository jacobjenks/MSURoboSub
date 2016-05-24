#!/usr/bin/env python
import serial
import rospy
import re
from sensor_msgs.msg import Imu, Temperature

imuPort = '/dev/ttyUSB0'
imuBaud = 115200
ser

def imuTalker():
	pubImu = rospy.Publisher('imu', Imu, queue_size=10)
	pubTemp = rospy.Publisher('temp', Temperature, queue_size=10)
	rospy.init_node('imu')
	rate = rospy.Rate(100)#Update at GEDC-6E update rate

	imuMsg = Imu()
	imuMsg.header.seq = 0
	imuMsg.header.frame_id = 0

	#These need to be updated to reflect actual covariance
	imuMsg.orientation_covariance = [1,0,0,
									 0,1,0,
									 0,0,1]

	imuMsg.angular_velocity_covariance = [1,0,0,
										  0,1,0,
										  0,0,1]

	imuMsg.linear_acceleration_covariance = [1,0,0,
											 0,1,0,
											 0,0,1]


	tempMsg = Temperature()
	tempMsg.header.seq = 0
	tempMsg.header.frame_id = 0
	
	try: 
		ser = serial.Serial(imuPort, imuBaud)
	except serial.SerialException:
		rospy.logfatal("Error connecting to IMU.")


	ser.flush()#make sure buffer is empty before we start looping

	while not rospy.is_shutdown():
		
		imuMsg.header.stamp = rospy.get_time()
		imuMsg.header.seq += 1

		#Quaternion mag data
		data = getImuData("PSPA,QUAT\r\n")
		imuMsg.orientation.w = data[0]
		imuMsg.orientation.x = data[1]
		imuMsg.orientation.y = data[2]
		imuMsg.orientation.z = data[3]

		#Gyro data
		data = getImuData("PSPA,G\r\n")
		imuMsg.angular_velocity.x = data[0]
		imuMsg.angular_velocity.y = data[1]
		imuMsg.angular_velocity.z = data[2]
		
		#Accelerometer data
		data = getImuData("PSPA,A\r\n")
		imuMsg.linear_acceleration.x = data[0] 
		imuMsg.linear_acceleration.y = data[1]
		imuMsg.linear_acceleration.z = data[2]

		#Temperature data
		data = getImuData("PSPA,Temp\r\n")
		tempMsg.header.stamp = rospy.get_time()
		tempMsg.temperature = data[0]

		pubImu.publish(imuMsg)
		pubTemp.publish(tempMsg)

		rate.sleep()

def getImuData(command):
	ser.write(command)
	data = ser.readline()
	values = re.findall(r'[-\d.]', data)
	return values


if __name__ == '__main__':
	try:
		imuTalker()
	except rospy.ROSInterruptException:
		pass
