#!/usr/bin/env python

# Code by Chance Cardona on 3/6/2020.
# Follows a wall using Q-Learning,
# Utilized for CSCI-473 for Project 2

import rospy, math, time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D 
import numpy as np
import matplotlib.pyplot as plt

#Global position values
x, y, theta = 0, 0, 0

#Physical parameters
dmin = 0.5

#Q Learning Variables:
sMap = {'front-close':0, 'front-medium':1, 'front-far':2,
            'left-close':3, 'left-medium':4, 'left-far':5,
            'right-close':6, 'right-medium':7, 'right-far':8}
aMap = {'turnLeft':0, 'turnRight':1, 'goForward':2, 'turn180':3}
states = [0,1,2,3,4,5,6,7,8,9] #Define set of States
actions = [0,1,2, 3] #Define set of Actions
Q = np.array(np.zeros((len(states), len(actions)))) 
Q[sMap['front-close']][aMap['turnLeft']] = 1
Q[sMap['right-close']][aMap['goForward']] = 1
Q[sMap['left-close']][aMap['goForward']] = 1

#def 

#def chooseAction(Q, states, ranges): 
    


#Subscriber callback func. Updates position of Triton robot.
def laserCallback(msg):
    global ranges_
    ranges_ = msg.ranges#{
            #'right': min(msg.ranges[])
    #print('Ranges:', ranges_, 'Min Angle:', msg.angle_min, 'Max Angle', msg.angle_max)
    


def move(dist, speed):
    global x, y, theta
    x0, y0, theta0 = x, y, theta

    #define message. Using Pose2D to define how to move.
    vel_msg = Pose2D()

    #define topic and then publisher.
    loop_rate = rospy.Rate(10)
    cmd_vel_topic = '/triton_lidar/vel_cmd'
    vel_pub = rospy.Publisher(cmd_vel_topic, Pose2D, queue_size=2)

    while math.hypot(x-x0, y-y0) < dist:
        x += speed*math.cos(theta)
        y += speed*math.sin(theta)
        vel_msg.x, vel_msg.y = x, y
        vel_pub.publish(vel_msg)
        #print("X", x, "Y", y)
                
    #stop the robot when distance is reached.
    rospy.loginfo("reached")
    vel_pub.publish(vel_msg)



if __name__ == '__main__':
    try:
        global ranges_
        #Initialize node to the ROS Master
        rospy.init_node('triton_lidar_pose')      
        
        #declare pose subscriber
        position_topic = '/scan'
        pose_subscriber = rospy.Subscriber(position_topic, LaserScan, laserCallback) 

        time.sleep(2)

        #Initialize Q Table
        

        for i in range(5):
            move(2.0, 0.3)
            time.sleep(4)
        
        modelState = '{model_state: { model_name: triton_lidar, pose: { position: { x: 2.86134281893, y: 0.125907141799, z: 4.37897966708e-05 }, orientation: { x: -0.000146901110306, y: -0.000169459146978, z: 0.000185462525381, w: 0.999999957654 }}, twist: {  linear: { x: 10.4999990463, y: 0.0038952359464, z: 0.0 }, angular: { x: 0.0, y: 0.0, z: 0.0 } }, reference_frame: world }}'
        rospy.ServiceProxy('/gazebo/set_model_state', modelState) 

        plt.polar(np.linspace(0, 2*np.pi, len(ranges_)), ranges_)
        plt.show()

        #Draw M logo
        #move2goal(2.5, 4)

        #loop_rate = rospy.Rate(10)
        #rospy.spin()


    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
        pass

