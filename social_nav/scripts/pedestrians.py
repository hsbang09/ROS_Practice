#!/usr/bin/env python


import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist


import roslib
import math
import tf
import geometry_msgs.msg
import turtlesim.srv

import random


NUM_PEDESTRIANS = 3

def talker():
    
#    pubs = []
    
#    for i in range(num_pedestrians):
#        # Create separate publishers for each turtle pedestrian
#        pub = rospy.Publisher('/turtle{0}/cmd_vel'.format(i), Twist, queue_size=10)
#        pubs.append(pub)
    
    
    # 1. Generate nodes: publisher and subscriber for each turtle. 
    # 2. 
    
    
    rospy.init_node('pedestrians', anonymous=True)
    
    #listener = tf.TransformListener()

    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    spawner(random.random(), random.random(), random.random(), 'turtle1')
    
#    
#    for i in range(NUM_PEDESTRIANS):
#        spawner(random.random(), random.random(), random.random(), 'turtle{0}'.format(i))

    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        
        rospy.loginfo("hello world")
        
        rate.sleep()

            


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

