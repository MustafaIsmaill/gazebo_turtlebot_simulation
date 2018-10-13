#!/usr/bin/env python

from gazebo_robot import *
import rospy

if __name__ == '__main__':
    try:

        turtlebot = gazebo_robot("turtlebot_move", "turtlebot", -10.0, 0.0, 0.0)
    	rospy.sleep(3)

    	duration = rospy.Duration(0.5)
    	turtlebot.applyJointEffort('turtlebot::create::left_wheel', 1, duration)
    	turtlebot.applyJointEffort('turtlebot::create::right_wheel', 1, duration)

    except rospy.ROSInterruptException:
        print("exception occured")
