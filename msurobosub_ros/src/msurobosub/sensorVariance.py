#!/usr/bin/env python
import rospy
import numpy as np
import atexit
from sensor_msgs.msg import Imu
from msurobosub.msg import Depth
import inspect
import StringIO

sensorReadings = dict() 

def saveVariance():
	global sensorReadings
	f = open('/home/robosub/MSURoboSub/msurobosub_ros/src/msurobosub/calibration/sensor_variance.txt', 'w')
	for key in sorted(sensorReadings):
		f.write("%s variance: %.3f\n" % (key, np.var(sensorReadings[key])))
		f.write("%s average: %.3f\n" % (key, np.average(sensorReadings[key])))
		f.write("%s min: %.3f\n" % (key, np.min(sensorReadings[key])))
		f.write("%s max: %.3f\n" % (key, np.max(sensorReadings[key])))

#Recursively dig through sensor message and record all readings
def processReading(reading, key = ""):
	skip = ['deserialize_numpy', '_type', 'deserialize', '_has_header', '_slot_types', 'serialize', '_full_text', '__module__', '_md5sum', '_get_types', '__slots__', 'header', '__doc__', 'serialize_numpy', '__init__']

	recurse_whitelist = ['orientation', 'angular_velocity', 'linear_acceleration']
	terminal_whitelist = ['x', 'y', 'z', 'w', 'psi', 'depth']

	values = reading.__class__.__dict__.keys()
	for v in values:
		if v in recurse_whitelist:
			processReading(getattr(reading, v), v)
		elif v in terminal_whitelist:
			recordReading(key + "_" + v, getattr(reading, v))

def recordReading(key, value):
	global sensorReadings
	if key not in sensorReadings:
		sensorReadings[key] = np.array([value]) 
	else:
		sensorReadings[key] = np.append(sensorReadings[key], value)


def init():
	rospy.init_node('sensorVariance')
	atexit.register(saveVariance)
	subImu = rospy.Subscriber('sensors/imu', Imu, processReading)
	subDepth = rospy.Subscriber('sensors/depth', Depth, processReading)
	rospy.spin()
	

if __name__ == '__main__':
	try:
		init()
	except rospy.ROSInterruptException:
		pass
