#!/usr/bin/env python

import rospy

import roslib
import tf

from std_msgs.msg import String
from geometry_msgs.msg import Twist

import turtlesim.srv
from turtlesim.msg import Pose


from hlpr_speech_msgs.msg import StampedString

import random
import math
import numpy as np

NUM_PEDESTRIANS = 7
RATE = 10    # 10Hz
PERIOD = float(1)/RATE


#
#
#class Turtle():
#    
#    def __init__(self,name,i,x,y,theta, pedestrian = True, goal=None):
#        
#        self.name=name
#        self.id=i
#        self.start = {"x":x,"y":y,"theta":theta}
#        self.pedestrian = pedestrian
#        
#        # Maximum velocity
#        self.max_vel = 2
#        
#        # Acceleration term
#        self.desired_vel = 1 + random.uniform(-0.5,0.5)
#        self.relaxation_time = 1.5
#        
#        self.mass = 8
#        self.goal = goal
#        
#        
#        if self.pedestrian:  
#            self.velocity_publisher = rospy.Publisher('/turtle{0}/cmd_vel'.format(self.id), Twist, queue_size=10)            
#        else:
#            # User-controlled turtle (no publisher)
#            self.velocity_publisher = None
#        
#        
#        self.pose_subscriber = rospy.Subscriber('/turtle{0}/pose'.format(self.id), Pose, self.set_position)
#        self.pose = Pose()
#    
#    
#    def set_position(self, data):
#        self.pose = data
#        self.pose.x = round(self.pose.x, 5)
#        self.pose.y = round(self.pose.y, 5)
#    
#    def get_distance(self,x,y):
#        dist_x = abs(x-self.pose.x)
#        dist_y = abs(y-self.pose.y)
#        return math.sqrt(pow(dist_x,2)+pow(dist_y,2))
#    
#    def get_position(self):
#        return self.pose
#    
#    def stop(self):
#        cmd = Twist()
#        cmd.linear.x = 0
#        cmd.linear.y = 0
#        cmd.linear.z = 0
#        cmd.angular.x = 0
#        cmd.angular.y = 0
#        cmd.angular.z = 0
#        self.velocity_publisher.publish(cmd)
#    
#    
#    def move(self, objects):
#        
#        if not self.pedestrian:
#            # If the current turtle is turtle1, skip moving (turtle1 is user-controlled)
#            pass
#        
#        else:
#
#            goal_vector = np.array([self.goal[0],self.goal[1]]) - np.array([self.pose.x,self.pose.y])
#            
#            F1 = self.apply_acceleration_term()
#            F2 = self.apply_attractive_force(self.goal[0],self.goal[1])
#            F3 = self.apply_repulsive_force(objects)*30
#            
#            F = F1+F2+F3
#
#            if np.linalg.norm(goal_vector) > 0.5:
#                
#                acceleration = F/float(self.mass)
#                theta = self.pose.theta
#                speed = self.pose.linear_velocity
#                                
#                velocity = np.array([speed*math.cos(theta),speed*math.sin(theta)]) + PERIOD * acceleration
#
#                if np.linalg.norm(velocity) > self.max_vel:
#                    velocity = float(self.max_vel)/np.linalg.norm(velocity) * velocity
#                
#                linear = np.linalg.norm(velocity)
#                angular = 17*(math.atan2(velocity[1], velocity[0]) - self.pose.theta)                
#
#                cmd = Twist()
#                cmd.linear.x = linear
#                cmd.linear.y = 0
#                cmd.linear.z = 0
#                cmd.angular.x = 0
#                cmd.angular.y = 0
#                cmd.angular.z = angular
#
#                self.velocity_publisher.publish(cmd)
#            
#            else:
#                self.stop()
#            
#                
#        
        

        
        
def simulate_pedestrians():

    # Initialize node
    rospy.init_node('voice_command', anonymous=True)
    rospy.wait_for_service('spawn')
    
#    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    
#    objects = []
    
    # Spawn the first turtle, which is controlled by the user (using arrow key)
#    turtle1 = Turtle('turtle1',1,0,0,0,False)
#    objects.append(turtle1)
    
#    for i in range(NUM_PEDESTRIANS):
#        
#        pid = i+2
#        name = 'turtle{0}'.format(pid)
#        
#        # Set the initial position
#        init_x = random.uniform(0,1)
#        init_y = random.uniform(1,9)   
#        init_theta = random.uniform(-math.pi/2,math.pi/2)
#        
#        # Set the goal position
#        #goal_x = random.uniform(9,10)
#        goal_y = i+1
#        goal_x = 9
#        #goal_y = init_y
#        goal = np.array([goal_x,goal_y])
#        
#        objects.append(Turtle(name,pid,init_x,init_y,init_theta,True,goal))
#        spawner(init_x,init_y,init_theta, name)


    
    def print_msgs(data):
        
        print(data)   
    
    
    
    rate = rospy.Rate(RATE) # 10hz

    while not rospy.is_shutdown():
        
#        for i in range(len(objects)):
#            t = objects[i]
#            t.move(objects)
        
    

#        self.velocity_publisher = rospy.Publisher('/turtle{0}/cmd_vel'.format(self.id), Twist, queue_size=10)            
        
        command_subscriber = rospy.Subscriber('/hlpr_speech_commands', StampedString, print_msgs)
    
        rate.sleep()


            


if __name__ == '__main__':
    try:
        simulate_pedestrians()
    except rospy.ROSInterruptException:
        pass

