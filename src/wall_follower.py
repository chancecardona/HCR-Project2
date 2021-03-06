#!/usr/bin/env python

# Code by Chance Cardona on 3/6/2020.
# Follows a wall using Q-Learning,
# Utilized for CSCI-473 for Project 2

import rospy, tf, math, time, random
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, Pose2D, Twist
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, GetModelState
from std_srvs.srv import Empty
import matplotlib.pyplot as plt

class Agent(object):
    """Q Learning Agent."""

    def __init__(self):
        self.Q = {}      #Q is a dict with states (R, FR, F, L) being the key, and possible Action's rewards being the value.
        self.states = {'tooClose':0, 'close':1, 'medium':2, 'far':3, 'tooFar':4}
        self.x = np.linspace(-30,30,60)
        self.A = np.vstack([(self.x, np.ones(60))]).T
        a = len(self.states)
        #Pre initialize Q table so robot can follow walls.
        for r in range(a):
            for fr in [1,3]:
                for f in range(a-1):        #Wall Following preinitialized Q table (only enough to follow a wall)
                    for l in [1,3]:                 
                        for o in [0,1,2,-1]:
                            self.Q[(r,fr,f,l,o)] = np.array([-0.4, -0.5, -0.5]) #, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5])  #Move forwards
                            if (r >= 3):           
                                self.Q[(r,fr,f,l,o)][2] = -0.1 #turn right if R is too far
                            elif (r <= 1):
                                self.Q[(r,fr,f,l,o)][1] = -0.1 #turn left if R is too close
   

    #episodes to train on, alpha is learning rate.
    def train(self, robot, gazebo, episodes, alpha = 0.2, gamma = 0.8):
        for n in range(1, episodes):
            #Randomize Starting location
            robot.stop()
            randx = random.randrange(3,9) - 4.5
            randy = random.randrange(1,9) - 4.5
            gazebo.setModelState(randx, randy) #puts robot randomly in any square except top 2 rows. Always faces same direction.
            #Initialize Variables
            rospy.loginfo("Episode" + str(n))
            goodPolicy = 0
            stuck = 0
            prevState = self.getState(robot)
            prevModelState = gazebo.getModelState()
            for i in range(4000):
                self.policy(robot, prevState, n) #Take action with best Q value after observing the initial state
                curState = self.getState(robot)
                curModelState = gazebo.getModelState()
                self.Q[prevState] = self.Q[prevState] + alpha*( self.reward(prevState) + gamma*self.Q[curState] - self.Q[prevState] )
                #Create exit conditions##############
                #Stuck condition
                if np.isclose(curModelState.position.x, prevModelState.position.x, atol=0.002) and np.isclose(curModelState.position.y, prevModelState.position.y, atol=0.002): #Tests if robot has been stuck for 3 steps
                    stuck += 1
                    if stuck == 3:
                        rospy.loginfo("Robot stuck. Ending episode.")
                        break
                else:
                    stuck = 0
                #Good Policy condition
                if curState[0] == 2 and curState[2] != 0: #Tests if robot has found a good policy (right is med, front isnt too close)
                    goodPolicy += 1
                    if goodPolicy == 1000:
                        rospy.loginfo("Good Policy Learned. Ending episode.")
                        break
                else:
                    goodPolicy = 0
                #Tipped condition
                (r, p, y) = tf.transformations.euler_from_quaternion([curModelState.orientation.x, curModelState.orientation.y, curModelState.orientation.z, curModelState.orientation.w])
                if abs(r) > math.pi/8 or abs(p) > math.pi/8:
                    rospy.loginfo("Robot Tipped. Ending episode.")
                    break
                #Update states.
                prevState = curState
                prevModelState = curModelState
                if i % 50 == 0:
                    rospy.loginfo("Step: " + str(i))
                robot.loop_rate.sleep()
            self.saveQ()
        rospy.loginfo("Episodes completed. Q table learned. Saving file to '~/.ROS/ccardona_Q_table.npy'")


    def reward(self, state): 
        #Avoid states where right is too close, right is too far, front is too close, or left is close
        if state[0] == 0 or state[0] == 4 or state[2] == 0  or state[3] == 1:
            #rospy.loginfo("BAD")
            return -1
        else:
            #rospy.loginfo("MEH")
            return 0    
    

    #Epsilon Greedy policy. n is episode number.
    def policy(self, robot, state, n, epsilon0 = 0.9, d = 0.985):
        possibleActions = self.Q[state]

        #epsilon greedy with a decayed epsilon
        epsilon = epsilon0*(d**n)
        p = random.uniform(0,1)
        #rospy.loginfo(str(epsilon) + ' , ' + str(p))
        if p > epsilon:
            #rospy.loginfo("Max Move")
            maxInd = np.argmax(possibleActions)
        else:
            #rospy.loginfo("Rand Move")
            maxInd = random.randrange(len(possibleActions))
        
        #call function chosen
        if maxInd == 0:                 #Forward is placed first because maxReward will always return first max listed.
            #print("Forwards")
            robot.moveForward()
        elif maxInd == 1:
            #print("Left40")
            robot.turnLeft(np.radians(40))
        elif maxInd == 2:
            #print("Right40")
            robot.turnRight(np.radians(40))
        #elif maxInd == 2:
        #    #print("Left20")
        #    robot.turnLeft(np.radians(20))
        #elif maxInd == 3:
        #    #print("Left10")
        #    robot.turnLeft(np.radians(10))
        #elif maxInd == 4:
        #    #print("Left0")
        #    robot.turnLeft(np.radians(0.3))
        #elif maxInd == 5:
        #    #print("Right40")
        #    robot.turnRight(np.radians(40))
        #elif maxInd == 6:
        #    #print("Right20")
        #    robot.turnRight(np.radians(20))
        #elif maxInd == 7:
        #    #print("Right10")
        #    robot.turnRight(np.radians(10))
        #elif maxInd == 8:
        #    #print("Right0")
        #    robot.turnRight(np.radians(0.3))
    

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
        elif robot.ranges['front'] >= 0.6 and robot.ranges['front'] < 1.2:
            F = 2 #medium
        elif robot.ranges['front'] > 1.2:
            F = 3 #far
        #Defining Left
        if robot.ranges['left'] <= 0.5:
            L = 1
        elif robot.ranges['left'] > 0.5:
            L = 3
        #Defining Orientation
        self.sweep = robot.ranges['orientation']*np.cos(np.radians(self.x)) #Lidar values transformed into cartesian
        LS = np.linalg.lstsq(self.A, self.sweep, rcond=None) #Least Squares
        m, c = LS[0]
        r2 = 1 - LS[1][0] / (60 * np.var(self.sweep)) #residual values. 
        #rospy.loginfo(str(m) + ',' + str(r2)) #Printing. For testing purposes only.
        #rospy.loginfo(str(robot.ranges['orientation']))
        #plt.plot(self.x, self.sweep, 'o')
        #plt.plot(self.x, m*self.x + c)
        #plt.title(str(m) + ',' + str(r2))
        #plt.show()
        if r2 > 0.5:
            if m > 0.003:
                O = 0 #approaching
            elif m < -0.003:
                O = 2 #moving away
            else:
                O = 1 #parallel
        else:
            O = -1

        
        return (R, FR, F, L, O)

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
        try:
            #rospy.ServiceProxy('/gazebo/pause_physics', Empty)
            rospy.wait_for_service('/gazebo/get_model_state')
            #rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        except rospy.ServiceException as e:
            rospy.loginfo("GAZEBO NOT UPDATED")
            pass
        get_state_response = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState) 
        get_state = get_state_response('triton_lidar', 'world')
        return get_state.pose
    
    #Only set x,y positions (don't want to lift robot), and z orientation (don't want to tip robot)
    #Default values put robot in corner of maze.
    def setModelState(self, x = 3.5, y = 3.1, z = math.pi - 0.2):
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
                'right': min(msg.ranges[-90:-30]),      #or use np.mean
                'front-right': min(msg.ranges[-60:-30]),
                'front': min(np.concatenate( (msg.ranges[-15:], msg.ranges[:15]) )), #-30:30 originally
                'left': min(msg.ranges[30:90]),
                'orientation': msg.ranges[-120:-60],
                }
        #print('Right', self.ranges['right'], 'Left', ranges['left'], 'front', ranges['front'])
        
    def moveForward(self, speed = 0.3):
        #define message. Using Pose to control velocity.
        self.vel_msg.x = speed
        self.vel_msg.y = 0
        self.vel_msg.theta = 0
        #rospy.loginfo("Forwards")
        self.vel_pub.publish(self.vel_msg)
    
    def turnLeft(self, angSpeed, speed = 0.3):
        self.vel_msg.x = speed
        self.vel_msg.theta = angSpeed
        #rospy.loginfo("Turning Left")
        self.vel_pub.publish(self.vel_msg)
    
    def turnRight(self, angSpeed, speed = 0.3):
        self.vel_msg.x = speed
        self.vel_msg.theta = -angSpeed
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
      
        #Continue from Last saved state. Comment if you wanna start over.
        #Q_Agent.loadQ()

        isTraining = rospy.get_param('/train')

        if isTraining:
            #Train Maze
            rospy.loginfo("Starting in Train mode.")
            Q_Agent.train(triton, G, 150)
            print("Maze completed. Trying sample solution.")
        elif not isTraining:
            #Load predefined Maze
            rospy.loginfo("Starting in Test mode. Loading pretrained Q table.")
            try:
                Q_Agent.loadQ()
                print("Loaded Q table.")
            except:
                rospy.loginfo("No trained Q table. Starting from preinitialized Q table.")


        #Solve Maze
        G.setModelState()
        while True:
            state = Q_Agent.getState(triton)
            print(state)
            Q_Agent.policy(triton, state, 0, 0) #runs q table according to epsilon greedy.
            triton.loop_rate.sleep()


    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated.")
        pass

