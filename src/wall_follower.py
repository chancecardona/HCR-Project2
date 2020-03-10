#!/usr/bin/env python

# Code by Chance Cardona on 3/6/2020.
# Follows a wall using Q-Learning,
# Utilized for CSCI-473 for Project 2

import rospy, math, time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D
import matplotlib.pyplot as plt 

#Global position values
x, y, theta = 0, 0, 0

#Physical parameters
dmin = 0.5

#Q Learning Variables:
states1 = {front, left, right} 
states2 = {close, medium, far}
actions = {forward, left, right} #Define set of Actions



#Subscriber callback func. Updates position of Triton robot.
def laserCallback(msg):
    global ranges_
    ranges_ = msg.ranges
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
        Q = {
        (left, close) : right(), (left, medium) : forward, (left, far) : forward, 
        (right, close) : left, (right, medium) : forward, (right, far) : forward, 
        (front, close) : left, (front, medium) : forward, (front, far) : forward
        }
        

        for i in range(5):
            move(2.0, 0.3)
            time.sleep(4)

        #Draw M logo
        #move2goal(2.5, 4)

        #loop_rate = rospy.Rate(10)
        #rospy.spin()


    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
        pass

