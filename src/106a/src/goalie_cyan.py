#!/usr/bin/env python
import rospy
import numpy as np
from realtimepseudoAstar import plan
from globaltorobotcoords import transform
from nubot_common.msg import ActionCmd, VelCmd, OminiVisionInfo, BallInfo, ObstaclesInfo, RobotInfo
from nubot_common.msg import BallIsHolding
import sys

# For plotting
# import math
# import matplotlib.pyplot as plt

# Initialize publisher and rate
pub = 0
if int(sys.argv[1]) == 0:
    pub = rospy.Publisher('/NuBot1/nubotcontrol/actioncmd', ActionCmd, queue_size=1)
else: 
    pub = rospy.Publisher('/rival1/nubotcontrol/actioncmd', ActionCmd, queue_size=1)
rospy.init_node('pubsub', anonymous=True)
hertz = 10
rate = rospy.Rate(hertz)

goalie_origin = np.array([-950, 0])
def line_to_goal(goal_pos, current_pos, obstacles):
    if exists_clear_path(goal_pos, current_pos, obstacles):
        return np.arctan2(target[1] - robot_pos[1], target[0] - robot_pos[0])
    return False

def exists_clear_path(goal_pos, current_pos, obstacles):
    AB = goal_pos - current_pos
    for o in obstacles:
        AC = o[:2] - current_pos
        AD = AB * np.dot(AC, AB) / np.dot(AB, AB)
        D = current_pos + AD
        if np.linalg.norm(D - o[:2]) <= o[2]:
            return False
    return True

def in_range(robot_pos, ball_pos, thresh=100):
    val = np.linalg.norm(robot_pos - ball_pos)
    print(thresh, val)
    return val < thresh

isholding = 0
def holding_callback(data):
    global isholding
    isholding = int(data.BallIsHolding)

def callback(data):
    
    #Get ball position in global frame
    b = data.ballinfo
    ball_pos = np.array([b.pos.x, b.pos.y])

    #Get robot position and heading in global frame
    r = data.robotinfo[0]
    robot_pos = np.array([r.pos.x, r.pos.y])
    theta = r.heading.theta

    #Get obstacle positions in global frame
    obstacles = data.obstacleinfo
    obstacle_list = np.empty((0,3), float)
    obstacle_list = np.concatenate((obstacle_list, np.array([[-1150, -125, 50]])))
    obstacle_list = np.concatenate((obstacle_list, np.array([[-1150, 125, 50]])))
    for p in obstacles.pos:
        obstacle_list = np.concatenate((obstacle_list, np.array([[p.x, p.y, 100]])))

    if isholding:
        print("Here");
        t = np.array([-700, 0]) 
        target = plan(t, robot_pos, obstacle_list, 100, 400)
        thetaDes = np.arctan2(target[1] - robot_pos[1], target[0] - robot_pos[0]) - theta
    
        #Convert target from global coordinate frame to robot coordinate frame for use by hwcontroller
        target = transform(target[0], target[1], robot_pos[0], robot_pos[1], theta)
        
        #Generate ActionCmd() and publish to hwcontroller
        action = ActionCmd()
        action.target.x = target[0]
        action.target.y = target[1]
        action.maxvel = 300
        action.handle_enable = 1
        action.target_ori = -theta
        pub.publish(action)
        rate.sleep()
        action.strength = 100
        action.shootPos = 1
        pub.publish(action)

    if in_range(robot_pos, ball_pos, 500) and in_range(ball_pos, goalie_origin, 600):
        #Generate target position and heading in global frame from real-time psuedo A-star path planning algorithm
        target = plan(ball_pos, robot_pos, obstacle_list, 100, 400)
        thetaDes = np.arctan2(target[1] - robot_pos[1], target[0] - robot_pos[0]) - theta
    
        #Convert target from global coordinate frame to robot coordinate frame for use by hwcontroller
        target = transform(target[0], target[1], robot_pos[0], robot_pos[1], theta)
        
        #Generate ActionCmd() and publish to hwcontroller
        action = ActionCmd()
        action.target.x = target[0]
        action.target.y = target[1]
        action.maxvel = 300
        action.handle_enable = 1
        action.target_ori = thetaDes
        pub.publish(action)
        rate.sleep()
    elif not in_range(robot_pos, goalie_origin, 100):
        #Generate target position and heading in global frame from real-time psuedo A-star path planning algorithm
        target = plan(goalie_origin, robot_pos, obstacle_list, 100, 400)
        thetaDes = np.arctan2(target[1] - robot_pos[1], target[0] - robot_pos[0]) - theta
    
        #Convert target from global coordinate frame to robot coordinate frame for use by hwcontroller
        target = transform(target[0], target[1], robot_pos[0], robot_pos[1], theta) 
        
        #Generate ActionCmd() and publish to hwcontroller
        action = ActionCmd()
        action.target.x = target[0]
        action.target.y = target[1]
        action.maxvel = 300
        action.handle_enable = 1
        action.target_ori = thetaDes
        pub.publish(action)
        rate.sleep()
    else:
        action = ActionCmd()
        action.target.x = 0
        action.target.y = 0
        pub.publish(action)
        rate.sleep()
        pass
        


    


def listener():
    robot = int(sys.argv[1])
    print(robot, type(robot))
    if robot == 0:
        rospy.Subscriber("/NuBot1/omnivision/OmniVisionInfo", OminiVisionInfo, callback, queue_size=1)
        rospy.Subscriber("/NuBot1/ballisholding/BallIsHolding", BallIsHolding, holding_callback, queue_size=1)
    elif robot == 1:
        rospy.Subscriber("/rival1/omnivision/OmniVisionInfo", OminiVisionInfo, callback, queue_size=1)
        rospy.Subscriber("/rival1/ballisholding/BallIsHolding", BallIsHolding, holding_callback, queue_size=1)
    else:
        print("Call 0 for cyan and 1 for magenta")

    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
