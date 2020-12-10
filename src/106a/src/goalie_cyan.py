#!/usr/bin/env python
import rospy
import numpy as np
from realtimepseudoAstar import plan
from globaltorobotcoords import transform
from nubot_common.msg import ActionCmd, VelCmd, OminiVisionInfo, BallInfo, ObstaclesInfo, RobotInfo

# For plotting
# import math
# import matplotlib.pyplot as plt

# Initialize publisher and rate
pub = rospy.Publisher('/NuBot1/nubotcontrol/actioncmd', ActionCmd, queue_size=1)
rospy.init_node('pubsub', anonymous=True)
hertz = 10
rate = rospy.Rate(hertz)

goalie_origin = np.array([-1100, 0])

def in_range(robot_pos, ball_pos, thresh=100):
    val = np.linalg.norm(robot_pos - ball_pos)
    print(thresh, val)
    return val < thresh

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
    for p in obstacles.pos:
        obstacle_list = np.concatenate((obstacle_list, np.array([[p.x, p.y, 100]])))

    if in_range(robot_pos, ball_pos, 450) and in_range(ball_pos, goalie_origin, 600):
        #Generate target position and heading in global frame from real-time psuedo A-star path planning algorithm
        target = plan(ball_pos, robot_pos, obstacle_list, 100, 400)
        thetaDes = np.arctan2(target[1] - robot_pos[1], target[0] - robot_pos[0])
    
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
    elif not in_range(robot_pos, goalie_origin, 75):
        #Generate target position and heading in global frame from real-time psuedo A-star path planning algorithm
        target = plan(goalie_origin, robot_pos, obstacle_list, 100, 400)
        thetaDes = np.arctan2(target[1] - robot_pos[1], target[0] - robot_pos[0])
    
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
        pass
        


    


def listener():
    rospy.Subscriber("/NuBot1/omnivision/OmniVisionInfo", OminiVisionInfo, callback, queue_size=1)

    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
