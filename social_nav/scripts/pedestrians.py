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

NUM_PEDESTRIANS = 2


class Turtle():
    
    def __init__(self,name,i,x,y,theta, pedestrian = True, goal=None):
        
        self.name=name
        self.id=i
        self.start = {"x":x,"y":y,"theta":theta}
        self.pedestrian = pedestrian
        self.goal = goal
        
        if self.pedestrian:  
            
            self.velocity_publisher = rospy.Publisher('/turtle{0}/cmd_vel'.format(self.id), Twist, queue_size=10)
            self._goal = np.array([self.goal[0],self.goal[1]]) # Intermediate goal
            
        else:
            # User-controlled turtle (no publisher, no goal)
            self.velocity_publisher = None
            self._goal = None
        
        
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
    
    def stop(self):
        cmd = Twist()
        cmd.linear.x = 0
        cmd.linear.y = 0
        cmd.linear.z = 0
        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = 0
        self.velocity_publisher.publish(cmd)
    
    
    def move(self, objects):
        
        if not self.pedestrian:
            # If the current turtle is turtle1, skip moving (turtle1 is user-controlled)
            pass
        
        else:
            #self.apply_repulsive_force(objects)

            #angular = 4 * math.atan2(trans[1], trans[0])
            #linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)

            #linear = np.linalg.norm(self._goal) + self.pose.linear_velocity
            #angular = (math.atan2(self._goal[1], self._goal[0]) - self.pose.theta)

            
            #self.go_to_goal()
            

            goal_vector = np.array([self.goal[0],self.goal[1]]) - np.array([self.pose.x,self.pose.y])
            
            print(self.goal)
            
            if np.linalg.norm(goal_vector) > 0.05:
                
                print(goal_vector)
                print(math.atan2(goal_vector[1],goal_vector[0]))
                
                linear = 0.5*np.linalg.norm(goal_vector)
                angular = 1*(math.atan2(goal_vector[1], goal_vector[0]) - self.pose.theta)

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
            
                
    
    
    def go_to_goal(self):
        pass
#        try:
#            
#            diff = (np.array([self.pose.x,self.pose.y]) - np.array([self.goal[0],self.goal[1]]))
#            
#            if np.linalg.norm(diff)==0:
#                print("{0} position: {1},{2}. diff: {3}".format(self.name,self.pose.x,self.pose.y,diff))
#
#            # Normalize to get direction
#            direction_to_goal = diff/np.linalg.norm(diff)
#            
#            
#            # Get distance to the goal
#            distance_to_goal = self.get_distance(self.goal[0], self.goal[1])
#
#
#
#            
#            self._goal = distance2Goal * direction2Goal
#        
#        except Exception as e:
#            print(e)
#    
    
    def apply_repulsive_force(self, objects):
        pass
#        try:
#            for o in objects:
#                
#                if o is self:
#                    # If the pedestrian is itself, pass
#                    continue
#
#                # Get the position of the object
#                position = o.get_position()
#                
#                # Get distance to the object
#                distance = self.get_distance(position.x, position.y)
#                
#                if distance < 1:
#                    
#                    diff = (np.array([self.pose.x,self.pose.y]) - np.array([position.x,position.y]))
#
#                    if np.linalg.norm(diff)==0:
#                        print("{0} position: {1},{2}. diff: {3}".format(self.name,self.pose.x,self.pose.y,diff))
#
#                    # Normalize to get direction
#                    direction = diff/np.linalg.norm(diff)
#                    
#                    # delta_linear_vel = math.exp(-distance)
#                    # self._goal = delta_linear_vel * direction
#                    self._goal = direction
#                    
#                
#                else:
#                    
#                    self._goal = np.array([0,0])
#        
#        except Exception as e:
#            print(e)
        
        
        
        
        
        
def simulate_pedestrians():

    # Initialize node
    rospy.init_node('pedestrians', anonymous=True)
    
    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    
    objects = []
    
    # Spawn the first turtle, which is controlled by the user (using arrow key)
    turtle1 = Turtle('turtle1',1,0,0,0,False)
    objects.append(turtle1)
    
    for i in range(NUM_PEDESTRIANS):
        
        pid = i+2
        name = 'turtle{0}'.format(pid)
        
        # Set the initial position
        init_x = random.uniform(0,2)
        init_y = random.uniform(3,7)   
        init_theta = random.uniform(-math.pi,math.pi)
        
        # Set the goal position
        goal_x = random.uniform(9,11)
        goal_y = random.uniform(0,10)
        goal = np.array([goal_x,goal_y])
        
        objects.append(Turtle(name,pid,init_x,init_y,init_theta,True,goal))
        spawner(init_x,init_y,init_theta, name)

        
    
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        
        for i in range(NUM_PEDESTRIANS):
            t = objects[i]
            t.move(objects)

        #rospy.loginfo()
        
        rate.sleep()


            


if __name__ == '__main__':
    try:
        simulate_pedestrians()
    except rospy.ROSInterruptException:
        pass

