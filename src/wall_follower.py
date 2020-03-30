#!/usr/bin/env python

# Code by Chance Cardona on 3/6/2020.
# Follows a wall using Q-Learning,
# Utilized for CSCI-473 for Project 2

import rospy, math, time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, Pose2D, Twist
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, GetModelState


class Agent(object):
    """Q Learning Agent."""

    def __init__(self):
        self.Q = {}      #Q is a dict with states (R, FR, F, L) being the key, and possible Action's rewards being the value.
        self.states = {'tooClose':0, 'close':1, 'medium':2, 'far':3, 'tooFar':4}
        a = len(self.states)
        #Pre initialize Q table.
        for i in range(a):
            for j in [1,3]:
                for k in range(a-1):
                    for l in [1,3]:
                        self.Q[(i,j,k,l)] = [0, 0.5, 0, 0]   #Move forwards normally
                        if (i <= 1):            #Unless R is tooClose then turn left
                            self.Q[(i,j,k,l)][2] = 1 
                        elif (i == 3):          #Or if R is Far then turn right.
                            self.Q[(i,j,k,l)][3] = 1
        
    def reward(self, state): 
        #Avoid states where right is too close, right is too far, front is too close, or left is close
        if state[0] == 0 or state[0] == 4 or state[2] == 0 or state[3] == 1:
            return -1
        return 0    
    
    def policy(self, robot, state):
        possibleActions = self.Q[state]
        maxInd = possibleActions.index(max(possibleActions))
        #call function chosen
        if maxInd == 0:
            robot.stop()
        elif maxInd == 1:
            robot.moveForward()
        elif maxInd == 2:
            robot.turnLeft()
        elif maxInd == 3:
            robot.turnRight()
    
    def getState(self, robot): 
        #Defining Right
        if robot.ranges['right'] < 0.5:
            R = 0 #too close
        elif robot.ranges['right'] >= 0.5 and robot.ranges['right'] < 0.6:
            R = 1 #close
        elif robot.ranges['right'] >= 0.6 and robot.ranges['right'] <= 0.8:
            R = 2 #medium
        elif robot.ranges['right'] > 0.8 and robot.ranges['right'] <= 1.2:
            R = 3 #far
        elif robot.ranges['right'] > 1.2:
            R = 4 #too far
        #Defining Front-Right
        if robot.ranges['front-right'] <= 1.2:
            FR = 1
        elif robot.ranges['front-right'] > 1.2:
            FR = 3
        #Defining Front
        if robot.ranges['front'] < 0.5:
            F = 0 #too close
        elif robot.ranges['front'] >= 0.5 and robot.ranges['front'] < 0.6:
            F = 1 #close
        elif robot.ranges['front'] >= 0.6 and robot.ranges['front'] <= 1.2:
            F = 2 #medium
        elif robot.ranges['front'] > 1.2:
            F = 3 #far
        #Defining Left
        if robot.ranges['left'] <= 1.2:
            L = 1
        elif robot.ranges['left'] > 1.2:
            L = 3
    
        return (R, FR, F, L)
    


class Gazebo(object):
    """Gazebo Functions. Allow us to get current state and set state of triton."""
    def __init__(self):
        self.state_msg = ModelState()
        self.state_msg.model_name = 'triton_lidar'
        self.state_msg.pose = self.getModelState()   #Pose object 
        self.setModelState() #Puts robot at corner of maze. can get rid of this.

    def getModelState(self):
        rospy.wait_for_service('/gazebo/get_model_state')
        get_state_response = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState) 
        get_state = get_state_response('triton_lidar', 'world')
        return get_state.pose
    
    #Only set x,y positions (don't want to lift robot), and z orientation (don't want to tip robot)
    #Default values put robot in corner of maze.
    def setModelState(self, x = 3.5, y = 3.1, z = math.pi/4 - 0.2):
        rospy.wait_for_service('/gazebo/set_model_state')
        self.state_msg.pose.position.y = y
        self.state_msg.pose.position.x = x
        self.state_msg.pose.orientation.z = z
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState) 
        resp = set_state(self.state_msg)

        


class Triton(object):
    """Class for controlling the triton robot."""

    def __init__(self):
        #Initialize node to the ROS Master
        rospy.init_node('triton_lidar_robot')      
        
        #declare lidar subscriber
        self.ranges = None
        self.position_topic = '/scan'
        self.pose_subscriber = rospy.Subscriber(self.position_topic, LaserScan, self.laserCallback) 
        
        #declare movement publisher
        self.cmd_vel_topic = '/triton_lidar/vel_cmd'
        self.vel_pub = rospy.Publisher(self.cmd_vel_topic, Pose2D, queue_size=2)
        self.vel_msg = Pose2D()

        #declare action space
        self.actions = {'stop':0, 'moveForward':1, 'turnLeft':2, 'turnRight':3}

        self.loop_rate = rospy.Rate(10)
        #Wait for the Subscriber to get values before trying to read them to prevent errors.
        while self.ranges == None:
            self.loop_rate.sleep

    #Subscriber callback func. Updates position of Triton robot.
    def laserCallback(self, msg):    
        #These are for if m != 360. then your index
        #would be int(Angle*m/dTheta)
        #m = len(msg.self.ranges)
        #dTheta = msg.angle_increment * 180/np.pi
        self.ranges = {
                'right': min(msg.ranges[-30:]+msg.ranges[:30]),
                'front-right': min(msg.ranges[30:60]),
                'front': min(msg.ranges[30:90]),
                'front-left': min(msg.ranges[120:150]),
                'left': min(msg.ranges[150:-150]),
                }
        #print('Right', self.ranges['right'], 'Left', ranges['left'], 'front', ranges['front'])
        
    def moveForward(self, speed = 0.3):
        #define message. Using Pose to control velocity.
        self.vel_msg.x = 0
        self.vel_msg.y = speed
        self.vel_msg.theta = 0
        rospy.loginfo("Forwards")
        self.vel_pub.publish(self.vel_msg)
    
    def turnLeft(self, speed = 0.3):
        self.vel_msg.y = speed
        self.vel_msg.theta = speed
        rospy.loginfo("Turning Left")
        self.vel_pub.publish(self.vel_msg)
    
    def turnRight(self, speed = 0.3):
        self.vel_msg.y = speed
        self.vel_msg.theta = -speed
        rospy.loginfo("Turning Right")
        self.vel_pub.publish(self.vel_msg)
                
    def stop(self):
        self.vel_msg.x = 0
        self.vel_msg.y = 0
        self.vel_msg.theta = 0
        rospy.loginfo("Stopping")
        self.vel_pub.publish(self.vel_msg)


if __name__ == '__main__':
    try:
        #Initialize Triton Robot
        triton = Triton()
        time.sleep(5)

        #Initialize the Q Agent
        Q_Agent = Agent() #Predefined Q Values
        
        #Initialize Model Pose
        G = Gazebo()

        #Train Maze

        #Solve Maze
        while True:
            state = Q_Agent.getState(triton)
            print(state)
            Q_Agent.policy(triton, state)
            triton.loop_rate.sleep()
        


    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated.")
        pass

