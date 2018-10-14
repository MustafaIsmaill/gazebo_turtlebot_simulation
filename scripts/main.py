#!/usr/bin/env python

from gazebo_robot import *
import rospy
import numpy as np
from math import pi

if __name__ == '__main__':
	try:

		turtlebot = gazebo_robot("turtlebot_move", "turtlebot", 0.0, 0.0, 0.0, 0.0)
		rospy.sleep(1)

		# turtlebot.getModelState()

		turtlebot.goToGoal()

		# turtlebot.setVelocity(1.478, 0, 0)

	except rospy.ROSInterruptException:
		print("exception occured")
