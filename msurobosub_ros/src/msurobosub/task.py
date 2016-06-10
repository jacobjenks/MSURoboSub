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
	subTasks = None	
	targetObject
	state = TaskStatus.waiting
	controlPub = None
	name = "Task"

	def __init__(self, priority = 0, name = "Task", subTasks = array()):
		self.priority = priority
		self.name = name
		if !isinstance(subTasks, list):
			rospy.logError("Invalid subTask list")
		else:
			self.subTasks = subTasks
		rospy.init_node('Task', anonymous=True)
		self.controlPub = rospy.Publisher('controller/pose', Pose, queue_size=5)
		rospy.spin()

	#Callback function for any subscribers the task uses to monitor its own status
	def monitorStatus():
	
	#Run subtasks, then run self task
	def run(self):
		subTasksDone = True 

		#Run subtasks in order
		for task in self.subTasks:
			if task.status == TaskStatus.waiting
				subTasksDone = False
				break
			elif task.status == TaskStatus.ready:
				subTasksDone = False
				task.run()

		#Run this task if all subtasks are complete
		if subTasksDone:	
			self.runSelf()

	def runSelf(self):


def MaintainDepthTask(Task):
	depthSub = None 
	motorPub = None 
	depthThreshold = 1#Depth in meters at which this node will activate
	currentDepth = 0
	motorCommand = MotorCommand()

	def __init__(self)
		Task.__init__(self, 1, "MaintainDepth")
		self.depthSub = rospy.Subscriber("sensors/depth", Depth, monitorStatus)
		self.status = TaskStatus.complete

	def monitorStatus(self, depth):
		if depth.depth < depthThreshold:
			self.state = TaskStatus.ready
		self.currentDepth = depth.depth

	def runSelf(self):
		if currentDepth > depthThreshold:
			self.state = TaskState.complete
		else:
			#TODO: Set motors to go down
			

class MoveToTask(Task):

	currentPose = None		#base_link Pose
	targetOdom = None		#nav_msgs/Odometry we want to move to relative to target frame
	targetFound = False		#Whether or not we know where the target is yet
	speed = 0				#Speed %
	proximityThresh = .1	#Distance (meters) threshold for arriving at target
	bumpThresh = 0 			#Acceleration threshold (m/s^2) for detecting a collision
	imuSub = None
	odomSub = None
	targetSub = None
	
	def __init__(self, priority, name, subTasks, speed, targetOdom, proximityThresh, bumpThresh):
		Task.__init__(self, priority, name, subTasks)
		self.imuSub = rospy.Subscriber("sensors/imu", Imu, self.monitorStatus)
		self.odomSub = rospy.Subscriber("odometry/filtered", Odometry, self.monitorStatus)
		self.targetSub = rospy.Subscriber("worldmodel/object/pose", PoseWithCovariance, self.monitorStatus)
		self.speed = speed
		self.targetOdom = targetOdom
		

	def monitorStatus(self, msg):
		if type(msg).__name__ == "Imu":
			if self.bumpThresh > 0 and (math.abs(msg.angular_velocity.x) > self.bumpThresh or math.abs(msg.angular_velocity.y > self.bumpThresh)
				self.status = TaskStatus.complete
		elif type(msg).__name__ == "Odometry":
			self.currentPose = msg.pose
			#TODO: Check distance from targetPose
		elif type(msg).__name__ == "PoseWithCovariance":
			if self.targetFound == False:
				self.targetFound == True
				self.status = TaskStatus.ready
			targetPose = msg.pose
			
	def runSelf(self):
		#TODO: send targetPose to controller


class PneumaticTask(Task):

	pneuPub = None
	pneuMsg = None

	def __init__(self, priority, name, subTasks, pneuID)
		'''
		@param pneuID: ID of the pneumatic device to activate
		'''
		Task.__init__(self, name, priority, subTasks)	
		self.pneuPub = rospy.Publisher("command/pneumatic", PneumaticCommand, queue_size=5)
		self.pneuMsg = PneumaticCommand()
		self.pneuMsg.command = pneuID

	def runSelf(self):
		self.pneuMsg.header.seq = 0
		self.pneuMsg.header.frame_id = "base_link"
		self.pneuMsg.header.stamp = rospy.get_rostime()
		pneuPub.publish(self.pneuMsg)
		
class TestBuoyTask(Task):

	def __init__(self):
		subTasks = array()
		subTasks.append(MaintainDepthTask())

		poseMsg = Pose()	
		poseMsg.header.seq = 0
		poseMsg.header.frame_id = "buoy_red"
		poseMsg.header.stamp = rospy.get_rostime()
		poseMsg.pose.position.x = 0
		poseMsg.pose.position.y = 0
		poseMsg.pose.position.z = 0

		subTasks.append(MoveToTask(2, "Buoy", array(), 1, poseMsg, .1, 1))

		Task.__init__(self, "Mission", "1", subTasks)

		#TODO: Create competition mission, and specific testing missions

'''
class CompetitionMissionTask(Task):
	def __init__(self):
		

'''


if __name__ == '__main__':
	try:
		TestBuoyTask()	
	except rospy.ROSInterruptException:
		pass
