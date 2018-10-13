#!/usr/bin/env python

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import ApplyJointEffort
from gazebo_msgs.srv import GetJointProperties
from gazebo_msgs.srv import GetModelState
import rospy
import time

class gazebo_robot:

	def __init__(self, node_name, name, X, Y, yaw): #initial robot conditions

		self._name = name
		self._x = X
		self._y = Y
		self._yaw = yaw

		rospy.init_node(node_name, anonymous=True)

		self._modelStateSetter = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=10, latch=True)
		rospy.sleep(1)
		self._jointPropertiesGetter = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
		rospy.wait_for_service('/gazebo/get_joint_properties')
		self._jointEffortApplier = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
		rospy.wait_for_service('/gazebo/apply_joint_effort')
		self._modelStateGetter = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
		rospy.wait_for_service('/gazebo/get_model_state')

		self._msg = ModelState()

		self._msg.model_name = name
		
		self._msg.pose.position.x = X
		self._msg.pose.position.y = Y
		self._msg.pose.position.z = 0.0
		self._msg.pose.orientation.x = 0.0
		self._msg.pose.orientation.y = 0.0
		self._msg.pose.orientation.z = 0.0
		self._msg.pose.orientation.w = 0.0
		self._msg.twist.linear.x = 0.0
		self._msg.twist.linear.y = 0.0
		self._msg.twist.linear.z = 0.0
		self._msg.twist.angular.x = 0.0
		self._msg.twist.angular.y = 0.0
		self._msg.twist.angular.z = yaw
		self._msg.reference_frame = "world"

		self._modelStateSetter.publish(self._msg)

	def getJointProperties(self, joint_name):

		joints = self._jointPropertiesGetter(joint_name)
		return joints

	def applyJointEffort(self, joint_name, effort, duration):

		start_time = rospy.get_rostime()
		self._jointEffortApplier(joint_name, effort, start_time, duration)

	def setPosition(self, X, Y, yaw):

		self._msg.pose.position.x = X
		self._msg.pose.position.y = Y
		self._msg.twist.angular.z = yaw

		self._modelStateSetter.publish(self._msg)

	def setVelocity(self, x_dot, y_dot, yaw):

		self._msg.twist.linear.x = x_dot
		self._msg.twist.linear.y = y_dot
		self._msg.twist.angular.z = yaw

		self._modelStateSetter.publish(self._msg)

	def getDistance(self, goal_x, goal_y):

		distance = sqrt(pow((goal_x - self._x), 2) + pow((goal_y - self._y), 2))
		return distance

	def getModelState(self):

		state = self._modelStateGetter(self._name, "world")
		print(state)
