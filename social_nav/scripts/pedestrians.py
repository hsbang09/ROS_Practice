#!/usr/bin/env python

import rospy
import turtlesim.srv
import geometry_msgs.msg
import tf
import math
import roslib

def talker():

    rospy.init_node('pedestrians',anonymous=True)
    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)

    rate = rospy.Rate(10) # 10 hz

    while not rospy.is_shutdown():
        
        rospy.loginfo("hello world")

        rate.sleep()


if __name__ == '__main__':

    try:
        talker()

    except rospy.ROSInterruptException:
        pass
