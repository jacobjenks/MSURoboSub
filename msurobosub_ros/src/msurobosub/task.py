#!/usr/bin/env python
import rospy
import math
from enum import Enum
from msurobosub/msgs import Depth 
from geometry_msgs import Pose, PoseWithCovariance
from sensor_msgs import Imu
from nav_msgs import Odometry


class TaskStatus(Enum):
	waiting = 1		#Waiting for prerequisites to be fullfilled
	ready = 2		#Ready to run
	complete = 3	#Task complete

class Task:
	priority = 0
	subTasks = array()	
	targetObject
	state = TaskStatus.waiting
	controlPub = None

	def __init__(self, priority = 0):
		global taskID
		self.priority = priority
		rospy.init_node('Task', anonymous=True)
		self.controlPub = rospy.Publisher('controller/pose', Pose, queue_size=5)
		rospy.spin()

	#Callback function for any subscribers the task uses to monitor its own status
	def monitorStatus():
	
	def run(self):
		#Run subtasks in order
		for task in self.subTasks:
			if task.status == TaskStatus.waiting
				break
			elif task.status == TaskStatus.ready:
				task.run()

		#Run this task
		self.runSelf()

	def runSelf(self):


def MaintainDepthTask(Task):
	depthSub = None 
	motorPub = None 
	depthThreshold = 1#Depth in meters at which this node will activate
	currentDepth = 0
	motorCommand = MotorCommand()

	def monitorStatus(self, depth):
		if depth.depth < depthThreshold:
			self.state = TaskStatus.ready
		self.currentDepth = depth.depth

	def __init__(self):
		Task.__init__(self, 1)
		self.depthSub = rospy.Subscriber("sensors/depth", Depth, monitorStatus)
		self.status = TaskStatus.complete

	def runSelf(self):
		if currentDepth > depthThreshold:
			self.state = TaskState.complete
		else:
			#TODO: Set motors to go down
			

class MoveToTask(Task):

	currentPose = None		#base_link Pose
	targetFrame = None		#TF frame for targetPose	
	targetPose = None		#Pose we want to move to relative to target frame
	targetFound = False		#Whether or not we know where the target is yet
	speed = 0				#Speed %
	proximityThresh = .1	#Distance (meters) threshold for arriving at target
	bumpThresh = 0 			#Acceleration threshold (m/s^2) for detecting a collision
	imuSub = None
	odomSub = None
	targetSub = None

	def monitorStatus(self, msg):
		if type(msg).__name__ == "Imu":
			if self.bumpThresh > 0 and any(math.abs(x) > self.bumpThresh for x in msg.angular_velocity)
				self.status = TaskStatus.complete
		elif type(msg).__name__ == "Odometry":
			self.currentPose = msg.pose
			#TODO: Check distance from targetPose
		elif type(msg).__name__ == "PoseWithCovariance":
			if self.targetFound == False:
				self.targetFound == True
				self.status = TaskStatus.ready
			targetPose = msg.pose
	
	def __init__(self, targetObject, positionOffset):
		Task.__init__(self)
		imuSub = rospy.Subscriber("sensors/imu", Imu, monitorStatus)
		odomSub = rospy.Subscriber("odometry/filtered", Odometry, monitorStatus)
		targetSub = rospy.Subscriber("worldmodel/object/pose", PoseWithCovariance, monitorStatus)
		
	def runSelf(self):
		#TODO: send targetPose to controller

if __name__ == '__main__':
	try:
		#imuTalker()
	except rospy.ROSInterruptException:
		pass
