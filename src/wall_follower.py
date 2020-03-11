#!/usr/bin/env python

# Code by Chance Cardona on 3/6/2020.
# Follows a wall using Q-Learning,
# Utilized for CSCI-473 for Project 2

import rospy, math, time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, GetModelState
import numpy as np

#Global position values
x, y, theta = 0, 0, 0
ranges_ = None

#Physical parameters
dmin = 0.5
dmax = 1.2

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
    

#Gazebo Functions. Allow us to get current state and set state of triton.
def getState():
    rospy.wait_for_service('/gazebo/set_model_state')
    get_state_response = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState) 
    get_state = get_state_response('triton_lidar', 'world')
    return get_state.pose

def setState(pose):
    rospy.wait_for_service('/gazebo/set_model_state')
    state_msg = ModelState()
    state_msg.model_name = 'triton_lidar'
    state_msg.pose = pose
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState) 
    resp = set_state(state_msg)

        



#Subscriber callback func. Updates position of Triton robot.
def laserCallback(msg):
    global ranges_
    #These are for if m != 360. then your index
    #would be int(Angle*m/dTheta)
    #m = len(msg.ranges)
    #dTheta = msg.angle_increment * 180/np.pi
    ranges_ = {
            'right': max(min(min(msg.ranges[-90:-30]), dmax), dmin),
            'front-right': min(msg.ranges[-60:-30]),
            'front': max(min(min(msg.ranges[-30:]+msg.ranges[:30]), dmax), dmin),
            'front-left': min(msg.ranges[30:60]),
            'left': max(min(min(msg.ranges[30:90]), dmax), dmin),
            }

    #print('Right', ranges_['right'], 'Left', ranges_['left'], 'front', ranges_['front'])
    


def moveForward(dist, speed):
    #define message. Using Twist to control velocity.
    #vel_msg = Pose2D()
    vel_msg = Twist()
    
    #define topic and then publisher.
    cmd_vel_topic = '/triton_lidar/vel_cmd'
    vel_pub = rospy.Publisher(cmd_vel_topic, Pose2D, queue_size=2)
    loop_rate = rospy.Rate(10)

    while:
        x = speed*math.sin(theta)
        y += speed*math.cos(theta)
        vel_msg.x, vel_msg.y = x, y
        vel_pub.publish(vel_msg)
        #print("X", x, "Y", y)
                
    #stop the robot when distance is reached.
    rospy.loginfo("reached")
    vel_pub.publish(vel_msg)


if __name__ == '__main__':
    try:
        #Initialize node to the ROS Master
        rospy.init_node('triton_lidar_robot')      
        
        #declare pose subscriber
        position_topic = '/scan'
        pose_subscriber = rospy.Subscriber(position_topic, LaserScan, laserCallback) 

        time.sleep(4)

        #Initialize Q Table
        

        move(2.0, 0.3)
        time.sleep(2)
       
#        pose = getState()
#        pose.position.x = 3
#        pose.orientation.z = -np.pi/4
#        setState(pose)
#        print('Right', ranges_['right'], 'Left', ranges_['left'], 'front', ranges_['front'])

        

        
        #loop_rate = rospy.Rate(10)
        #rospy.spin()


    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
        pass

