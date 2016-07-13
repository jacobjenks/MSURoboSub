#!/usr/bin/env python
import rospy
from hector_object_tracker.msgs import ImagePercept
from sensor_msgs.msgs import Image

msgImagePercept = None
pubImagePercept = None
gpuReady = True

#Pixel ratio for our webcam
#This is the number of pixels for an object 1 meter wide at a distance of 1 meter
pixelRatio = 1280

objects = None

class Color:
	r = 0
	g = 0
	b = 0

	def __init__(self, r, g, b):
		self.r = r
		self.g = g
		self.b = b

class Object:
	id = 0 #frcnn id
	name = ""
	width = 0
	height = 0
	color = None 

	def __init__(self, id, width, height, color):
		self.id = id
		self.width = width 
		self.height = height
		self.color = color

def subImageCB(imageMsg):
	global gpuReady
	
	if !gpuReady:
		return

	gpuReady = False	
	fasterRCNN(imageMsg)
	gpuReady = True

def fasterRCNN(imageMsg):
	global pubImagePercept
	#Run the thing

	msgImagePercept.header.seq += 1
	msgImagePercept.header.stamp = rospy.get_rostime()
	msgImagePercept.x = 0
	msgImagePercept.y = 0
	msgImagePercept.distance = 5
	pubImagePercept.publish(msgImagePercept)


#Estimate the distance to an object using its angular diameter (in pixels),
# and a pixel ratio calibrated for our specific camera
def estimateDistance(xMin, xMax, objectClass):
	global objects, pixelRatio

	for o in objects:
		if o.id == objectClass:
			return pixelRatio/o.width/(xMax - xMin)
	
	
def initObjects():
	global objects
	objects = list()
	buoy, gate, bin, bin_cover, torpedo_target, torpedo_hole, torpedo_cover, object_pickup, object_dropoff, path_marker, pinger
	objects.append(Object(1, "qual gate", 3.05, 1.52, Color(255, 69, 0)))
	objects.append(Object(2, "red buoy", 0.2, 0.24, Color(255, 0, 0)))
	objects.append(Object(2, "green buoy", 0.2, 0.24, Color(0, 255, 0)))
	objects.append(Object(2, "yellow buoy", 0.2, 0.24, Color(255, 255, 0)))
	objects.append(Object(3, "channel", 2.4, 1.2, Color(255, 255, 0)))
	objects.append(Object(4, "bin", 0.3, 0.6, Color(255, 255, 0)))
	objects.append(Object(5, "bin_cover", 0.3, 0.6, Color(0, 0, 0)))
	objects.append(Object(6, "torpedo target", 1.2, 1.2, Color(255, 255, 0)))
	objects.append(Object(7, "torpedo hole", 0.3, 0.3, Color(0, 0, 0)))
	objects.append(Object(8, "torpedo cover", 0.25, 0.25, Color(0, 0, 0)))
	objects.append(Object(9, "path marker", 1, 0.2, Color(0, 0, 0)))#Fix size

def objectDetector():
	global pubImagePercept, msgImagePercept
	pubImagePercept = rospy.Publisher('worldmodel/image_percept', ImagePercept, queue_size=10)
	rospy.init_node('objectDetector')
	rospy.Subscriber("sensors/camF/image_raw", Image, subCamCB)

	msgImagePercept = ImagePercept()
	msgImagePercept.header.seq = 0
	msgImagePercept.header.frame_id = "camF"

	initObjects()

	rospy.spin()


if __name__ == '__main__':
	try:
		objectDetector()
	except rospy.ROSInterruptException:
		pass
