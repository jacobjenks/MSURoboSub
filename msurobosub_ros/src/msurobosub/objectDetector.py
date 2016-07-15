#!/usr/bin/env python
import rospy
import cv2
from hector_object_tracker.msgs import ImagePercept
from sensor_msgs.msgs import Image

msgImagePercept = None
pubImagePercept = None

imageSkip = 7 #Only use every nth image
currentImage = 0

#Pixel ratio for our webcam
#This is the number of pixels for an object 1 meter wide at a distance of 1 meter
pixelRatio = 1280
objectDefs = None # Object definitions - color, size, etc
lastCamImage = None

CvBridge = CvBridge()

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

	#Estimate the distance to an object using its angular diameter (in pixels),
	# and a pixel ratio calibrated for our specific camera
	def estimateDistance(self, yMin, yMax):
		global pixelRatio
		return pixelRatio/self.height/(yMax - yMin)


def subImageCB(imageMsg):
	global imageSkip, currentImage, CvBridge

	if currentImage % imageSkip == 0:
		objects = fasterRCNN(imageMsg)	
		drawObjects(CvBridge.imgmsg_to_cv2(imageMsg), objects)
		
	currentImage += 1
	

# Run faster rcnn, publish messages for object detections, and return array of object detections
def fasterRCNN(imageMsg):
	global pubImagePercept, objectDefs
	#Run the thing

	#objects = RUN

	for o in objects:
		msgImagePercept = ImagePercept()
		msgImagePercept.header = imageMsg.header
		msgImagePercept.camera_info = None #TODO
		msgImagePercept.x = (o.xMin + o.xMax) / 2 #Center point of object
		msgImagePercept.y = (o.yMin + o.yMax) / 2 
		msgImagePercept.width = (o.xMax - o.xMin) / msgImagePercept.camera_info.width
		msgImagePercept.height = (o.yMax - o.yMin) / msgImagePercept.camera_info.height
		msgImagePercept.distance = objectDefs[o.id].estimateDistance(o.yMin, o.yMax)
		pubImagePercept.publish(msgImagePercept)

	return objects

def drawObjects(image, objects):
	global pubObjectDetector, objectDefs, CvBridge

	image = image.clone()

	for o in objects:
		cv2.rectangle(image, (o.xMin, o.yMin), (o.xMax, o.yMax), (0, 255, 0))
		cv2.putText(image, objectDefs[o.id].name + ":" + objectDefs[o.id].estimateDistance(o.yMin, o.yMax), FONT_HERSHEY_SIMPLEX, 1, (0,255,0)) 

	pubObjectDetector.publish(CvBridge.bridge.cv2_to_imgmsg(image))

def initObjects(): 
	global objects
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
	global pubImagePercept
	pubImagePercept = rospy.Publisher('worldmodel/image_percept', ImagePercept, queue_size=10)
	pubObjectDetector = rospy.Publisher('object_detector', Image, queue_size=10)
	rospy.Subscriber("sensors/camF/image_raw", Image, subCamCB)
	rospy.init_node('objectDetector')

	initObjects()

	rospy.spin()


if __name__ == '__main__':
	try:
		objectDetector()
	except rospy.ROSInterruptException:
		pass
