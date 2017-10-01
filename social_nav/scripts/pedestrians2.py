#!/usr/bin/env python

import rospy

from std_msgs.msg import String
from geometry_msgs.msg import Twist

import turtlesim.srv
from turtlesim.msg import Pose

import random
import math
import numpy as np

NUM_PEDESTRIANS = 20
RATE = 10    # 10Hz
PERIOD = float(1)/RATE
PID = 1

class Turtle():
    
    def __init__(self,name,i,x,y,theta, pedestrian = True, goal=None):
        
        self.name=name
        self.id=i
        self.start = {"x":x,"y":y,"theta":theta}
        self.pedestrian = pedestrian
        
        # Maximum velocity
        self.max_vel = random.uniform(0.3,1)
        
        # Acceleration term
        self.desired_vel = 0.35 + random.uniform(-0.2,0.2)
        self.relaxation_time = random.uniform(1,3)
        
        self.mass = random.uniform(50,150)
        self.goal = goal
        
        
        if self.pedestrian:  
            self.velocity_publisher = rospy.Publisher('/turtle{0}/cmd_vel'.format(self.id), Twist, queue_size=10)            
        else:
            # User-controlled turtle (no publisher)
            self.velocity_publisher = None
        
        
        self.pose_subscriber = rospy.Subscriber('/turtle{0}/pose'.format(self.id), Pose, self.set_position)
        self.pose = Pose()
    
    
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
	
	killed = False        

        if not self.pedestrian:
            # If the current turtle is turtle1, skip moving (turtle1 is user-controlled)
            pass
        
        else:

            goal_vector = np.array([self.goal[0],self.goal[1]]) - np.array([self.pose.x,self.pose.y])
            
            F1 = self.apply_acceleration_term()
            F2 = self.apply_attractive_force(self.goal[0],self.goal[1])
            F3 = self.apply_repulsive_force(objects)
            
            F = F1+F2+F3

            if np.linalg.norm(goal_vector) > 0.5:
                
                acceleration = F/float(self.mass)
                theta = self.pose.theta
                speed = self.pose.linear_velocity
                                
                velocity = np.array([speed*math.cos(theta),speed*math.sin(theta)]) + PERIOD * acceleration

                if np.linalg.norm(velocity) > self.max_vel:
                    velocity = float(self.max_vel)/np.linalg.norm(velocity) * velocity
                
                linear = np.linalg.norm(velocity)
                angular = 15*(math.atan2(velocity[1], velocity[0]) - self.pose.theta)                

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
		
                rospy.wait_for_service('kill')
                kill = rospy.ServiceProxy('kill', turtlesim.srv.Kill)
                kill(self.name)
                killed = True
	
        return killed
        
        
    def apply_acceleration_term(self):
        
        # Desired direction
        e_a = np.array([self.goal[0],self.goal[1]]) - np.array([self.pose.x,self.pose.y])
        
        # Desired speed
        v0_a = self.desired_vel
        
        desired_velocity = v0_a * e_a
        
        theta = self.pose.theta
        speed = self.pose.linear_velocity
        
        current_velocity = np.array([speed*math.cos(theta),speed*math.sin(theta)])
        
        F_a = float(1)/self.relaxation_time * (desired_velocity - current_velocity)
        
        return F_a
        
        
    def apply_attractive_force(self, att_x, att_y):

        r_i = np.array([att_x,att_y])
        r_a = np.array([self.pose.x,self.pose.y])
        r_ai = r_a - r_i
                
        # Potential: e^r
        R = np.linalg.norm(r_ai)
        f_ai_x = r_ai[0] * math.exp(R) / R
        f_ai_y = r_ai[1] * math.exp(R) / R
        f_ai = - np.array([f_ai_x, f_ai_y])
        
        return f_ai
            

    def apply_repulsive_force(self, objects):
        
        F = np.array([0,0])
            
        try:
            for o in objects:
                
                if o is self:
                    # If the pedestrian is itself, pass
                    continue

                # Get the position of the object
                position = o.get_position()
                
                r_b = np.array([position.x,position.y])
                r_a = np.array([self.pose.x,self.pose.y])
                r_ab = r_a - r_b

                
                if np.linalg.norm(r_ab)==0:
                    pass
		    #print("{0} and {1} have the same position".format(o.name,self.name))
                
                # Potential: 1/r
#                f_ab_x = - r_ab[0] / math.pow(np.linalg.norm(r_ab),3)
#                f_ab_y = - r_ab[0] / math.pow(np.linalg.norm(r_ab),3)
                
                # Potential: e^(-r)
                R = np.linalg.norm(r_ab)
                f_ab_x = - r_ab[0] / float(0.5 * math.exp(R/float(0.5)) * R)
                f_ab_y = - r_ab[1] / float(0.5 * math.exp(R/float(0.5)) * R)
                f_ab = - 45 * np.array([f_ab_x, f_ab_y])
                F = F + f_ab
        
        except Exception as e:
            print(e)
        
        
        return F
        
        
        
def simulate_pedestrians():

    # Initialize node
    rospy.init_node('pedestrians', anonymous=True)
    
    rospy.wait_for_service('/turtle1/teleport_absolute')
    teleport = rospy.ServiceProxy('/turtle1/teleport_absolute', turtlesim.srv.TeleportAbsolute)
    teleport(5.5,0,math.pi/float(2))

    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    
    
    objects = []
    
    rate = rospy.Rate(RATE) 

    # Spawn the first turtle, which is controlled by the user (using arrow key)
    turtle1 = Turtle('turtle1',1,0,0,0,False)
    PID=1
    objects.append(turtle1)

    while not rospy.is_shutdown():
	
        existing = len(objects)
        for i in range(NUM_PEDESTRIANS-existing):

	    PID=PID+1
	    name = 'turtle{0}'.format(PID)

	    # Set the initial and goal position

	    if PID % 2 == 0: #even 
	        init_x = random.uniform(0,1)
	        init_y = random.uniform(0,11) 
	        goal_y = random.uniform(0,11)
	        goal_x = 10
		init_theta = random.uniform(-math.pi/2,math.pi/2)
	    else:
	    	init_x = random.uniform(10,11)
	    	init_y = random.uniform(0,11) 
	    	goal_y = random.uniform(0,11)
	    	goal_x = 0
		init_theta = random.uniform(math.pi/2,math.pi*3/2)

	    goal = np.array([goal_x,goal_y])
	    objects.append(Turtle(name,PID,init_x,init_y,init_theta,True,goal))
	    spawner(init_x,init_y,init_theta, name)

	survived = []
        for i in range(len(objects)):
            t = objects[i]
            killed=t.move(objects)
	    if not killed:
		survived.append(t)
	
	objects = survived         

        rate.sleep()


            


if __name__ == '__main__':
    try:
        simulate_pedestrians()
    except rospy.ROSInterruptException:
        pass

