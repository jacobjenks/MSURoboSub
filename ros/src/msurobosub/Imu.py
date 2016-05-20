#!/usr/bin/env python
import serial
import rospy
from sensor_msgs import Imu
from msurobosub import Temp

imuPort = '/dev/ttyUSB0'
imuBaud = 115200

def imuTalker():
	pubImu = rospy.Publisher('imu', Imu, queue_size=10)
	pubTemp = rospy.Publisher('temp', Float64, queue_size=10)
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


	tempMsg = Temp()
	tempMsg.header.seq = 0
	tempMsg.header.frame_id = 0
	
	try: 
		ser = serial.Serial(imuPort, imuBaud)
	except serial.SerialException:
		rospy.logfatal("Error connecting to IMU.")

	ser.flush()#make sure buffer is empty before we start looping

	while not rospy.is_shutdown():
		data = ser.readline()
		if len(data) > 0:
			data = data.split(',')
			imuMsg.header.stamp = rospy.get_time()
			imuMsg.header.seq += 1

			imuMsg.orientation.x = data[0]
			imuMsg.orientation.y = data[1]
			imuMsg.orientation.z = data[2]
			imuMsg.orientation.w = data[3]
			imuMsg.angular_velocity.x = data[4]
			imuMsg.angular_velocity.y = data[5]
			imuMsg.angular_velocity.z = data[6]
			imuMsg.linear_acceleration.x = data[7] 
			imuMsg.linear_acceleration.y = data[8]
			imuMsg.linear_acceleration.z = data[9]

			tempMsg.header.stamp = rospy.get_time()
			tempMsg.temp = data[10]

			pubImu.publish(imuMsg)
			pubTemp.publish(tempMsg)
		rate.sleep()

if __name__ == '__main__':
	try:
		imuTalker()
	except rospy.ROSInterruptException:
		pass
