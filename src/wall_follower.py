#!/usr/bin/env python

# Code by Chance Cardona on 3/6/2020.
# Follows a wall using Q-Learning,
# Utilized for CSCI-473 for Project 2

import rospy, math, time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D, Twist
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, GetModelState
import numpy as np

#Global values
ranges_ = None
Q = {} #Q has key states (R, FR, F, L), value action rewards (as long as actions is).

#Q Learning Variables:
sMap = {'tooClose':0, 'close':1, 'medium':2, 'far':3, 'tooFar':4}
aMap = {'stop':0, 'moveForward':1, 'turnLeft':2, 'turnRight':3}


def train(states):
    global Q
    a = len(states)
    #Just go straight always
    for i in range(a):
        for j in [1,3]:
            for k in range(a-1):
                for l in [1,3]:
                    Q[(i,j,k,l)] = [0,1,0,0]
                    if ((k == 0 or k == 1 or k == 2 or j == 3):
                        Q[(i,j,k,l)][2] = 2 #if front is close-med we or right-front is close we always turn left.
    



#def reward(Q, state):
#    #Avoid states where any of the sensors is too close or too far
#    for s in state:
#        if s == 0 or s == 4
#            Q[state] = -1
#
#    Q[] = 1
#    Q[] = 1 


def policy(actions, state):
    possibleActions = Q[state]
    maxInd = possibleActions.index(max(possibleActions))
    #call function chosen
    if maxInd == 0:
        stop()
    elif maxInd == 1:
        moveForward()
    elif maxInd == 2:
        turnLeft()
    elif maxInd == 3:
        turnRight()



def getState(): 
    #Defining Right
    if ranges_['right'] < 0.5:
        R = 0 #too close
    elif ranges_['right'] >= 0.5 and ranges_['right'] < 0.6:
        R = 1 #close
    elif ranges_['right'] >= 0.6 and ranges_['right'] <= 0.8:
        R = 2 #medium
    elif ranges_['right'] > 0.8 and ranges_['right'] <= 1.2:
        R = 3 #far
    elif ranges_['right'] > 1.2:
        R = 4 #too far
    #Defining Front-Right
    if ranges_['front-right'] <= 1.2:
        FR = 1
    elif ranges_['front-right'] > 1.2:
        FR = 3
    #Defining Front
    if ranges_['front'] < 0.5:
        F = 0 #too close
    elif ranges_['front'] >= 0.5 and ranges_['front'] < 0.6:
        F = 1 #close
    elif ranges_['front'] >= 0.6 and ranges_['front'] <= 1.2:
        F = 2 #medium
    elif ranges_['front'] > 1.2:
        F = 3 #far
    #Defining Left
    if ranges_['left'] <= 1.2:
        L = 1
    elif ranges_['left'] > 1.2:
        L = 3

    return (R, FR, F, L)
    



#Gazebo Functions. Allow us to get current state and set state of triton.
def getModelState():
    rospy.wait_for_service('/gazebo/get_model_state')
    get_state_response = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState) 
    get_state = get_state_response('triton_lidar', 'world')
    return get_state.pose

def setModelState(pose):
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
            'right': min(msg.ranges[-90:-30]),
            'front-right': min(msg.ranges[-60:-30]),
            'front': min(msg.ranges[-30:]+msg.ranges[:30]),
            'front-left': min(msg.ranges[30:60]),
            'left': min(msg.ranges[30:90]),
            }

    #print('Right', ranges_['right'], 'Left', ranges_['left'], 'front', ranges_['front'])
    


#makes robot move forward.
def moveForward(speed = 0.3):
    #define message. Using Pose to control velocity.
    vel_msg = Pose2D()
    vel_msg.x = 0
    vel_msg.y = speed
    vel_msg.theta = 0
    rospy.loginfo("Forwards")
    vel_pub.publish(vel_msg)

def turnLeft(speed = 0.3):
    vel_msg = Pose2D()
    vel_msg.theta = speed
    rospy.loginfo("Turning Left")
    vel_pub.publish(vel_msg)

def turnRight(speed = 0.3):
    vel_msg = Pose2D()
    vel_msg.theta = -speed
    rospy.loginfo("Turning Right")
    vel_pub.publish(vel_msg)
            
#stop the robot
def stop():
    vel_msg = Pose2D()
    vel_msg.x = 0
    vel_msg.y = 0
    vel_msg.theta = 0
    rospy.loginfo("Stopping")
    vel_pub.publish(vel_msg)


if __name__ == '__main__':
    try:
        #Initialize node to the ROS Master
        rospy.init_node('triton_lidar_robot')      
        
        #declare lidar subscriber
        position_topic = '/scan'
        pose_subscriber = rospy.Subscriber(position_topic, LaserScan, laserCallback) 
        
        #declare movement publisher.
        cmd_vel_topic = '/triton_lidar/vel_cmd'
        vel_pub = rospy.Publisher(cmd_vel_topic, Pose2D, queue_size=2)
        
        #Set up loop / Let things cool off
        loop_rate = rospy.Rate(10)
        time.sleep(5)

        #Train the model (set our predefined Q values)
        actions = aMap #Define set of Actions
        s = sMap
        train(s)
        prevTime = time.time()

        while True:
            state = getState()
            print(state)
            policy(actions, state)
            time.sleep(0.1)
            #print(time.time() - prevTime)
        
        #rospy.spin()


    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated.")
        pass

