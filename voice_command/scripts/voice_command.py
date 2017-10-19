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

RATE = 10    # 10Hz
PERIOD = float(1)/RATE


TOP_RIGHT_CORNER = [11, 11]
TOP_LEFT_CORNER = [0, 11]
BOTTOM_LEFT_CORNER = [0, 0]
BOTTOM_RIGHT_CORNER = [11, 0]
CENTER = [5.5, 5.5]



class Turtle():
    
    def __init__(self, name, i):
        
        self.name = name
        self.id = i
        
        self.velocity_publisher = rospy.Publisher('/turtle{0}/cmd_vel'.format(self.id), Twist, queue_size=10)            
        self.pose_subscriber = rospy.Subscriber('/turtle{0}/pose'.format(self.id), Pose, self.set_position)
        self.pose = Pose()
    
    
    def set_position(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 5)
        self.pose.y = round(self.pose.y, 5)
    
    def get_distance(self, x, y):
        dist_x = abs(x-self.pose.x)
        dist_y = abs(y-self.pose.y)
        return math.sqrt(pow(dist_x,2)+pow(dist_y,2))
    
    def get_position(self):
        return self.pose
    
    def stop(self):
        cmd = Twist()
        cmd.linear.x = 0
        cmd.linear.y = 0
        cmd.linear.z = 0
        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = 0
        self.velocity_publisher.publish(cmd)
    
    
    def move_to_goal(self, goal):

        goal_vector = np.array([goal[0],goal[1]]) - np.array([self.pose.x,self.pose.y])
        if np.linalg.norm(goal_vector) > 0.3:

            theta = self.pose.theta
            speed = self.pose.linear_velocity
            velocity = np.array([speed*math.cos(theta),speed*math.sin(theta)])

            linear = np.linalg.norm(goal_vector)
            angular = math.atan2(goal_vector[1], goal_vector[0]) - self.pose.theta               
            
            cmd = Twist()
            cmd.linear.x = linear
            cmd.linear.y = 0
            cmd.linear.z = 0
            cmd.angular.x = 0
            cmd.angular.y = 0
            cmd.angular.z = angular
            self.velocity_publisher.publish(cmd)

        else:
            self.stop()
            
                
    def process_command(self, data):
                
        command = data.keyphrase
        
        print('{0} received command: {1}'.format(self.name,command))
       
        if command == "TOP RIGHT":
            self.move_to_goal(TOP_RIGHT_CORNER)
        
        elif command == "TOP LEFT":            
            self.move_to_goal(TOP_LEFT_CORNER)
            
        elif command == "BOTTOM LEFT":
            self.move_to_goal(BOTTOM_LEFT_CORNER)
        
        elif command == "BOTTOM RIGHT":
            self.move_to_goal(BOTTOM_RIGHT_CORNER)
        
        elif command == "CENTER":
            self.move_to_goal(CENTER)
            
        else:
            #raise ValueError('I cannot understand the command!')
            print('I cannot understand the command!')
        
def run_voice_command():

    # Initialize node
    rospy.init_node('voice_command', anonymous=True)
    #rospy.wait_for_service('spawn')
        
    print('Initiating the voice command')

    # Spawn the first turtle, which is controlled by the user (using arrow key)
    turtle1 = Turtle('turtle1',1)

    rate = rospy.Rate(RATE) # 10hz

    while not rospy.is_shutdown():
        
        
#        self.velocity_publisher = rospy.Publisher('/turtle{0}/cmd_vel'.format(self.id), Twist, queue_size=10)            
        command_subscriber = rospy.Subscriber('/hlpr_speech_commands', StampedString, turtle1.process_command)
        rate.sleep()


            


if __name__ == '__main__':
    try:
        run_voice_command()
    except rospy.ROSInterruptException:
        pass

