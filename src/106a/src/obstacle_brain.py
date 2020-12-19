#!/usr/bin/env python
import rospy
import sys
import time
import numpy as np
from realtimepseudoAstar import plan
from globaltorobotcoords import transform
from nubot_common.msg import ActionCmd, VelCmd, OminiVisionInfo, BallInfo, ObstaclesInfo, RobotInfo, BallIsHolding


#Initialize desired x depending on obstacle number
ROBOT_NAME = 'rival' + str(sys.argv[1])
possible_x = [-600, -200, 200, 600]
target_1 = np.array([possible_x[int(sys.argv[1]) - 1], -400])
target_2 = np.array([possible_x[int(sys.argv[1]) - 1], 400])
current_target = target_1


# For plotting
# import math
# import matplotlib.pyplot as plt

# Initialize publisher and rate
pub = rospy.Publisher('/' + str(ROBOT_NAME)+'/nubotcontrol/actioncmd', ActionCmd, queue_size=1)
rospy.init_node(str(ROBOT_NAME) + '_brain', anonymous=False)
hertz = 10
rate = rospy.Rate(hertz)

def callback(data):

    #Receive all robot info    
    r = data.robotinfo[int(sys.argv[1]) - 1]
    robot_pos = np.array([r.pos.x, r.pos.y])
    theta = r.heading.theta

    #Alternate between +y and -y target positions

    global current_target

    if np.linalg.norm(robot_pos - current_target) < 50 and np.all(current_target == target_1):
        current_target = target_2
    elif np.linalg.norm(robot_pos - current_target) < 50 and np.all(current_target == target_2):
        current_target = target_1

    target = current_target

    #Convert target from global coordinate frame to robot coordinate frame for use by hwcontroller

    target = transform(target[0], target[1], robot_pos[0], robot_pos[1], theta)

    #Generate ActionCmd() and publish to hwcontroller
    action = ActionCmd()
    action.target.x = target[0]
    action.target.y = target[1]
    action.maxvel = 150
    action.handle_enable = 0
    action.target_ori = 0
    pub.publish(action)
    rate.sleep()


def listener():
    rospy.Subscriber("/" + str(ROBOT_NAME) + "/omnivision/OmniVisionInfo", OminiVisionInfo, callback, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
