#!/usr/bin/env python

# Code by Chance Cardona on 3/6/2020.
# Follows a wall using Q-Learning,
# Utilized for CSCI-473 for Project 2

import rospy, math, time, random
import numpy as np
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
        #Pre initialize Q table so robot can follow walls.
        for i in range(a):
            for j in [1,3]:
                for k in range(a-1):
                    for l in [1,3]:
                        self.Q[(i,j,k,l)] = np.array([0, 0, 0]) #np.array([0.5, 0, 0])   #Move forwards normally
                        #if (i <= 1):            #Unless R is tooClose then turn left
                        #    self.Q[(i,j,k,l)][1] = 1 
                        #elif (i == 3):          #Or if R is Far then turn right.
                        #    self.Q[(i,j,k,l)][2] = 1
    
    #episodes to train on, alpha is learning rate.
    def train(self, robot, gazebo, episodes, alpha = 0.2, gamma = 0.8):
        for n in range(episodes):
            #Randomize Starting location
            robot.stop()
            randx = random.randrange(1,9) - 4.5
            randy = random.randrange(1,9) - 4.5
            gazebo.setModelState(randx, randy) #puts robot randomly in any square. Always faces same direction.
            rospy.loginfo("Episode" + str(n))
            goodPolicy = 0
            stuck = 0
            prevState = self.getState(robot)
            prevModelState = gazebo.getModelState()
            for i in range(10000):
                self.policy(robot, prevState, n) #Take action with best Q value after observing the initial state
                curState = self.getState(robot)
                curModelState = gazebo.getModelState()
                self.Q[prevState] = self.Q[prevState] + alpha*( self.reward(prevState) + gamma*max(self.Q[curState]) - self.Q[prevState] )
                #Create exit conditions
                if np.isclose(curModelState.position.x, prevModelState.position.x, atol=0.001) and np.isclose(curModelState.position.y, prevModelState.position.y, atol=0.001): #Tests if robot has been stuck for 30 steps
                    stuck += 1
                    if stuck == 3:
                        rospy.loginfo("Robot stuck. Ending episode.")
                        break
                else:
                    stuck = 0
                if curState[0] == 2 and curState[2] != 0: #Tests if robot has found a good policy (right is med, front isnt too close)
                    goodPolicy += 1
                    if goodPolicy == 1000:
                        rospy.loginfo("Good Policy Learned. Ending episode.")
                        break
                else:
                    goodPolicy = 0
                #Update states.
                prevState = curState
                prevModelState = curModelState
                #print(prevModelState.position.x, prevModelState.position.y)
                robot.loop_rate.sleep()
        rospy.loginfo("Episodes completed. Q table learned. Saving file to '~/.ROS/ccardona_Q_table.npy'")
        self.saveQ()


    def reward(self, state): 
        #Avoid states where right is too close, right is too far, front is too close, or left is close
        if state[0] == 0 or state[0] == 4 or state[2] == 0 or state[3] == 1:
            return -1
        return 0    
    

    #Epsilon Greedy policy. n is episode number.
    def policy(self, robot, state, n, epsilon0 = 0.9, d = 0.985):
        possibleActions = self.Q[state]

        #epsilon greedy with a decayed epsilon
        epsilon = epsilon0*(d**n)
        p = random.uniform(0,1)
        if p >= 1 - epsilon:
            maxInd = np.argmax(possibleActions)
        else:
            maxInd = random.randrange(len(possibleActions))
        
        #call function chosen
        if maxInd == 0:
            robot.moveForward()
        elif maxInd == 1:
            robot.turnLeft()
        elif maxInd == 2:
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

    def saveQ(self):
        np.save('ccardona_Q_table.npy', self.Q)

    def loadQ(self):
        self.Q = np.load('ccardona_Q_table.npy', allow_pickle='TRUE').item()
    


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
        #rospy.loginfo("Forwards")
        self.vel_pub.publish(self.vel_msg)
    
    def turnLeft(self, speed = 0.3):
        self.vel_msg.y = speed
        self.vel_msg.theta = speed
        #rospy.loginfo("Turning Left")
        self.vel_pub.publish(self.vel_msg)
    
    def turnRight(self, speed = 0.3):
        self.vel_msg.y = speed
        self.vel_msg.theta = -speed
        #rospy.loginfo("Turning Right")
        self.vel_pub.publish(self.vel_msg)
                
    def stop(self):
        self.vel_msg.x = 0
        self.vel_msg.y = 0
        self.vel_msg.theta = 0
        #rospy.loginfo("Stopping")
        self.vel_pub.publish(self.vel_msg)


if __name__ == '__main__':
    try:
        #Initialize Triton Robot
        triton = Triton()
        time.sleep(5)

        #Initialize the Q Agent
        Q_Agent = Agent() #Predefined Q Values
        
        #Initialize Model Pose.
        G = Gazebo()
        
        isTraining = rospy.get_param('/train')

        if isTraining:
            #Train Maze
            rospy.loginfo("Starting in Train mode.")
            Q_Agent.train(triton, G, 150)
        elif not isTraining:
            #Load predefined Maze
            rospy.loginfo("Starting in Test mode. Loading predefined Q table. Make sure this exists.")
            Q_Agent.loadQ()

        print("Maze completed. Trying sample solution.")

        #Solve Maze
        while True:
            state = Q_Agent.getState(triton)
            print(state)
            Q_Agent.policy(triton, state, 0, 1) #runs q table according to epsilon greedy.
            triton.loop_rate.sleep()
        


    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated.")
        pass

