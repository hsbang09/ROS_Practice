#!/usr/bin/env python

import rospy

import roslib
import tf

from std_msgs.msg import String
from geometry_msgs.msg import Twist

import turtlesim.srv
from turtlesim.msg import Pose

import random
import math
import numpy as np

NUM_PEDESTRIANS = 7

class Turtle():
    
    def __init__(self,name,i,x,y,theta, pedestrian = True):
        
        self.name=name
        self.id=i
        self.starting_point = {"x":x,"y":y,"theta":theta}
        self.pedestrian = pedestrian
        
        if self.pedestrian:
            self.velocity_publisher = rospy.Publisher('/turtle{0}/cmd_vel'.format(self.id), Twist, queue_size=10)
        else:
            self.velocity_publisher = None
        
        
        self.pose_subscriber = rospy.Subscriber('/turtle{0}/pose'.format(self.id), Pose, self.set_position)
        
        self.pose = Pose()
        self.rate = rospy.Rate(10) # 10 Hz
    
    
    def set_position(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 5)
        self.pose.y = round(self.pose.y, 5)
    
    def get_distance(self,x,y):
        dist_x = abs(x-self.pose.x)
        dist_y = abs(y-self.pose.y)
        return math.sqrt(pow(dist_x,2)+pow(dist_y,2))
    
    def get_position(self):
        return self.pose
    
    
    def move(self, objects):
        self.repulsive_force(objects)
    
    
    def repulsive_force(self, objects):
        

        try:
            for o in objects:
                
                if o is self:
                    # If the pedestrian is itself, pass
                    continue

                position = o.get_position()    
                
                # Get distance to another pedestrian
                distance = self.get_distance(position.x,position.y)

                if not self.pedestrian:
                    # If the current turtle is turtle1, pass
                    pass

                elif distance < 1.5:

                    diff = (np.array([self.pose.x,self.pose.y]) - np.array([position.x,position.y]))

                    if np.linalg.norm(diff)==0:
                        print("{0} position: {1},{2}. diff: {3}".format(self.name,self.pose.x,self.pose.y,diff))
                    
                    # Normalize to get direction
                    direction = diff/np.linalg.norm(diff)

                    
                    acceleration = distance*direction

                    #angular = 4 * math.atan2(trans[1], trans[0])
                    #linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)

                    linear = distance

                    angular = (math.atan2(direction[1], direction[0]) - self.pose.theta)

                    cmd = Twist()
                    cmd.linear.x = linear
                    cmd.angular.z = angular

                    self.velocity_publisher.publish(cmd)
        
        
        except Exception as e:
            print(e)
        
        
        
        
        
        
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
    
    objects = []
    
    turtle1 = Turtle('turtle1',1,0,0,0,False)
    objects.append(turtle1)
    
    for i in range(NUM_PEDESTRIANS):
        
        x = random.random()*3
        y = random.random()*6 + 3
        
        pid = i+2
        theta = random.random()*math.pi
        
        name = 'turtle{0}'.format(pid)
        
        objects.append(Turtle(name,pid,x,y,theta))
        spawner(x,y,theta, name)

    
    rate = rospy.Rate(10) # 10hz


    while not rospy.is_shutdown():
        
        for i in range(NUM_PEDESTRIANS):
            t = objects[i]
            t.move()

        #rospy.loginfo()
        
        rate.sleep()


            


if __name__ == '__main__':
    try:
        simulate_pedestrians()
    except rospy.ROSInterruptException:
        pass

