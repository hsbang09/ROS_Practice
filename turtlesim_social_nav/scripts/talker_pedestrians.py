#!/usr/bin/env python


import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist


import roslib
import math
import tf
import geometry_msgs.msg
import turtlesim.srv


num_pedestrians = 10

def talker():
    
#    pubs = []
    
#    for i in range(num_pedestrians):
#        # Create separate publishers for each turtle pedestrian
#        pub = rospy.Publisher('/turtle{0}/cmd_vel'.format(i), Twist, queue_size=10)
#        pubs.append(pub)
    
    
    # 1. Generate nodes: publisher and subscriber for each turtle. 
    # 2. 
    
    
    
    rospy.init_node('talker_pedestrians', anonymous=True)
    
    
    #listener = tf.TransformListener()

    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    
    for i in range(num_pedestrians):
        spawner(0, 0, 0, 'turtle{0}'.format(i))

    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        
        rospy.loginfo("hello world")
        
        rate.sleep()

            


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
