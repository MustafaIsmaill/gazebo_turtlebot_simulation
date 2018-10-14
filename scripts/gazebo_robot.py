#!/usr/bin/env python

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import ApplyJointEffort
from gazebo_msgs.srv import GetJointProperties
from gazebo_msgs.srv import GetModelState
from math import pow, atan2, sqrt
import rospy
import time

class gazebo_robot:

	def __init__(self, node_name, name, X, Y, Z_THETA, yaw): #initial robot conditions

		self._name = name
		self._node_name = node_name
		self._x = X
		self._y = Y
		self._z_theta = Z_THETA
		self._yaw = yaw

		rospy.init_node(self._node_name, anonymous=True)

		self._modelStateSetter = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=10, latch=True)
		rospy.sleep(1)
		self._jointPropertiesGetter = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
		rospy.wait_for_service('/gazebo/get_joint_properties')
		self._jointEffortApplier = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
		rospy.wait_for_service('/gazebo/apply_joint_effort')
		self._modelStateGetter = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
		rospy.wait_for_service('/gazebo/get_model_state')

		self._msg = ModelState()

		self._msg.model_name = self._name
		
		self._msg.pose.position.x = self._x
		self._msg.pose.position.y = self._x
		self._msg.pose.position.z = 0.0
		self._msg.pose.orientation.x = 0.0
		self._msg.pose.orientation.y = 0.0
		self._msg.pose.orientation.z = self._z_theta
		self._msg.pose.orientation.w = 0.0
		self._msg.twist.linear.x = 0.0
		self._msg.twist.linear.y = 0.0
		self._msg.twist.linear.z = 0.0
		self._msg.twist.angular.x = 0.0
		self._msg.twist.angular.y = 0.0
		self._msg.twist.angular.z = self._yaw
		self._msg.reference_frame = "world"

		self._modelStateSetter.publish(self._msg)

	def getJointProperties(self, joint_name):

		joints = self._jointPropertiesGetter(joint_name)
		return joints

	def applyJointEffort(self, joint_name, effort, duration):

		start_time = rospy.get_rostime()
		self._jointEffortApplier(joint_name, effort, start_time, duration)

	def setPosition(self, X, Y, Z_THETA):

		self._x = X
		self._y = Y
		self._z_theta = Z_THETA

		self._msg.pose.position.x = self._x
		self._msg.pose.position.y = self._y
		self._msg.pose.orientation.z = self._z_theta

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

	def goToGoal(self):
        
		rospy.sleep(2)

		goal_x = input("Input x goal:")
		goal_y = input("Input y goal:")
		distance_tolerance = 1

		while getDistance(goal_x, goal_y) >= distance_tolerance:

		    x_velocity = 1.5 * self.getDistance(goal_x, goal_y)
		    y_velocity = 1.5 * self.getDistance(goal_x, goal_y)
		    z_velocity = 0

		    yaw_velocity = 4 * (atan2(goal_y - self._y, goal_x - self._x) - self._yaw)

		    self.setVelocity(x_velocity, y_velocity, yaw_velocity)

		x_velocity = 0
		y_velocity = 0 
		yaw_velocity = 0

		self.setVelocity(x_velocity, y_velocity, yaw_velocity)