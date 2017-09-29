#!/usr/bin/env python

import rospy

import roslib
import tf

from std_msgs.msg import String
from geometry_msgs.msg import Twist

import turtlesim.srv
import turtlesim.msg import Pose

import random
import math


NUM_PEDESTRIANS = 3


class Pedestrian():
    
    def __init__(self,name,i,x,y,theta):
        self.name=name
        self.id=i
        self.starting_point = {"x":x,"y":y,"theta":theta}

        self.velocity_publisher = rospy.Publisher('/turtle{0}/cmd_vel'.format(self.id), Twist, queue_size=10)
        
        self.pose_subscriber = rospy.Subscriber('/turtle{0}/pose'.format(self.id), Pose, self.callback)
        self.pose = Pose()
        self.rate = rospy.Rate(10) # 10 Hz
    
    
    def setPosition(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)
        print('asdfads')
    
        
        
        
        
        
def simulate_pedestrians():
    
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
    
    pedestrians = []
    
    for i in range(NUM_PEDESTRIANS):
        x = random.random()*2
        y = random.random()*10
        theta = random.random()*math.pi
        pedestrians.append(Pedestrian(i,x,y,theta))
        spawner(x,y,theta, 'turtle{0}'.format(i+2))

    
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        
        rospy.loginfo("hello world")
        
        
        print('asdfasdfasdfadsf')
        
        rate.sleep()

            


if __name__ == '__main__':
    try:
        simulate_pedestrians()
    except rospy.ROSInterruptException:
        pass

