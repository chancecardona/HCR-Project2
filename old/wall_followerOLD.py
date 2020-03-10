#!/usr/bin/env python

# Code by Chance Cardona on 3/6/2020.
# Follows a wall using Q-Learning,
# Utilized for CSCI-473 for Project 2

import rospy, math, time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D

#Global position values
x, y, theta = 0, 0, 0

#Subscriber callback func. Updates position of Triton robot.
def laserCallback(msg):
    global ranges_
    ranges_ = msg.ranges
    print('Ranges:', ranges_, 'Min Angle:', msg.angle_min, 'Max Angle', msg.angle_max)



def move(speed, distance, angle=0, ang_speed=0):
    global x, y, theta
    x0, y0, theta0 = x, y, theta
    omega =  ang_speed * math.copysign(1, angle)

    #define message. Turtlesim uses Twist.
    velocity_message = Pose2D()
    velocity_message.linear.x = speed
    velocity_message.angular.z = math.radians(omega)

    #define topic and then publisher.
    loop_rate = rospy.Rate(10)
    cmd_vel_topic = '/triton_lidar/vel_cmd'
    velocity_publisher = rospy.Publisher(cmd_vel_topic, Pose2D, queue_size=10)

    distance_moved = 0.0
    cur_angle = 0
    t0 = rospy.Time.now().to_sec() 

    while cur_angle < abs(angle):
        rospy.loginfo("Rotating")
        velocity_publisher.publish(velocity_message)
        cur_angle = (rospy.Time.now().to_sec() - t0)*ang_speed

    while distance_moved < distance:
        rospy.loginfo("Moving Forwards")
        velocity_publisher.publish(velocity_message)
        distance_moved = (rospy.Time.now().to_sec() - t0)*speed
#        distance_moved += math.sqrt((x-x0)**2 + (y-y0)**2)
        loop_rate.sleep()

    #stop the robot when distance is reached.
    rospy.loginfo("reached")
    velocity_message.linear.x = 0
    velocity_message.anglular.z = 0
    velocity_publisher.publish(velocity_message)





def move2goal(relative_x, relative_y, lin_speed=1.5, ang_speed=6, m = 0.8 ): #m is just scaling
    #define Goal x coordinates
    global x, y, theta
    x0, y0, theta0 = x, y, theta
    goal_x = x0 + relative_x*m
    goal_y = y0 + (relative_y+0.001)*m #0.01 else angle gets off.

    #Define message, topic, and publisher
    vel_msg = Pose2D()
    cmd_vel_topic = '/triton_lidar/cmd_vel'
    velocity_publisher = rospy.Publisher(cmd_vel_topic, Pose2D, queue_size=10)
    #Define rate of loop
    loop_rate = rospy.Rate(10)
    i = 0

    #Simple closed loop PID
    while math.sqrt((goal_x - x)**2 + (goal_y - y)**2) > 0.01*m:
        #We want to turn before we begin moving
        if i > 20:
            vel_msg.linear.x = lin_speed * math.sqrt((goal_x - x)**2 + (goal_y - y)**2)
        vel_msg.angular.z = ang_speed * (math.atan2(goal_y - y, goal_x - x) - theta)
        i+=1
        rospy.loginfo(vel_msg.angular.z)
        velocity_publisher.publish(vel_msg)
        loop_rate.sleep()

    #Stop when goal is reached
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)
    #rospy.spin() #for ctrl-C
        



if __name__ == '__main__':
    try:
        #Initialize node to the ROS Master
        rospy.init_node('triton_lidar_pose')      
        
        #declare pose subscriber
        position_topic = '/scan'
        pose_subscriber = rospy.Subscriber(position_topic, LaserScan, laserCallback) 

        time.sleep(2)

        #Initialize Q Table
        #Q = {
        #(left, close) : right, (left, medium) : forward, (left, far) : forward, 
        #(right, close) : left, (right, medium) : forward, (right, far) : forward, 
        #(front, close) : left, (front, medium) : forward, (front, far) : forward
        #}
        #


        move(2.0, 0.3)
        #Draw M logo
        #move2goal(2.5, 4)

        #loop_rate = rospy.Rate(10)
        #rospy.spin()


    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
        pass

