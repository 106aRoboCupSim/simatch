#!/usr/bin/env python
import rospy
import sys
import time
import numpy as np
from realtimepseudoAstar import plan
from globaltorobotcoords import transform
from nubot_common.msg import ActionCmd, VelCmd, OminiVisionInfo, BallInfo, ObstaclesInfo, RobotInfo, BallIsHolding

ROBOT_NAME = 'NuBot' + str(sys.argv[1])
if str(sys.argv[2]) == '1':
    ROBOT_NAME = 'rival' + str(sys.argv[1])
opponent_goal = np.array([1100.0, 0.0])
isdribble = 0
shoot_range = 500

#Bare minimum is 50
obstacle_radius = 150
plan_radius = 500


# For plotting
# import math
# import matplotlib.pyplot as plt

# Initialize publisher and rate
pub = rospy.Publisher('/' + str(ROBOT_NAME)+'/nubotcontrol/actioncmd', ActionCmd, queue_size=1)
rospy.init_node(str(ROBOT_NAME) + '_brain', anonymous=False)
hertz = 20
rate = rospy.Rate(hertz)
#rate2 = rospy.Rate(1)

# For plotting path and path plan
# targets_generated_x = []
# targets_generated_y = []
# robot_position_x = []
# robot_position_y = []

# def plot_circle(x, y, size, color="-b"):  # pragma: no cover
#         deg = list(range(0, 360, 5))
#         deg.append(0)
#         xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
#         yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
#         plt.plot(xl, yl, color)

def callback(data):

    #Get ball position in global frame
    b = data.ballinfo
    ball_pos = np.array([b.pos.x, b.pos.y])
    if np.abs(ball_pos[0]) > 1100 and np.abs(ball_pos[1]) < 125:
        action = ActionCmd()
        action.target.x = 0
        action.target.y = 0
        action.maxvel = 0
        pub.publish(action)
        #print('sleeping')
        time.sleep(1.5)
        #rate2.sleep()

    #Get robot position and heading in global frame
    r = data.robotinfo[int(sys.argv[1]) - 1]
    robot_pos = np.array([r.pos.x, r.pos.y])
    theta = r.heading.theta

    #Get obstacle positions in global frame
    obstacles = data.obstacleinfo
    obstacle_list = np.empty((0,3), float)
    for p in obstacles.pos:
        obstacle_list = np.concatenate((obstacle_list, np.array([[p.x, p.y, obstacle_radius]])))
    #print(obstacle_list)
    #print(r.isdribble)
    target = plan(ball_pos, robot_pos, obstacle_list, plan_radius, 400, default_random=False)
    thetaDes = np.arctan2(target[1] - robot_pos[1], target[0] - robot_pos[0]) - theta
    #print(isdribble)
    action = ActionCmd()
    if isdribble and np.linalg.norm(opponent_goal - robot_pos) > shoot_range:
        target = plan(opponent_goal, robot_pos, obstacle_list, plan_radius, 400, default_random=False)
        thetaDes = np.arctan2(opponent_goal[1] - robot_pos[1], opponent_goal[0] - robot_pos[0]) - theta
    elif isdribble and np.linalg.norm(opponent_goal - robot_pos) < shoot_range:
        thetaDes = thetaDes = np.arctan2(opponent_goal[1] - robot_pos[1], opponent_goal[0] - robot_pos[0]) - theta
        target = robot_pos
        action.shootPos = 1
        action.strength = 200

    #Generate target position and heading in global frame from real-time psuedo A-star path planning algorithm
    # target = plan(ball_pos, robot_pos, obstacle_list, 100, 400)
    # thetaDes = np.arctan2(target[1] - robot_pos[1], target[0] - robot_pos[0])

    # For plotting
    # robot_position_x.append(robot_pos[0])
    # robot_position_y.append(robot_pos[1])
    # targets_generated_x.append(target[0])
    # targets_generated_y.append(target[1])

    #Convert target from global coordinate frame to robot coordinate frame for use by hwcontroller

    target = transform(target[0], target[1], robot_pos[0], robot_pos[1], theta)

    #Generate ActionCmd() and publish to hwcontroller
    #action = ActionCmd()
    action.target.x = target[0]
    action.target.y = target[1]
    action.maxvel = 300
    action.handle_enable = 1
    action.target_ori = thetaDes
    pub.publish(action)
    rate.sleep()


    #   # For plotting path and path plan of robot, after 100 pathing iterations
    # if len(targets_generated_x) > 100:
    #     fig, ax = plt.subplots()
    #     ax.scatter(targets_generated_x, targets_generated_y)
    #     ax.scatter(robot_position_x, robot_position_y)
    #     for i in range(len(targets_generated_x)):
    #         ax.annotate(i, (targets_generated_x[i], targets_generated_y[i]))
    #         ax.annotate(i, (robot_position_x[i], robot_position_y[i]))
    #     for o in obstacle_list:
    #         plot_circle(o[0], o[1], o[2])
    #     #print(targets_generated)
    #     plt.show()
    #     time.sleep(100)

def holdingballcallback(data):
    global isdribble
    isdribble = data.BallIsHolding


def listener():
    rospy.Subscriber("/" + str(ROBOT_NAME) + "/omnivision/OmniVisionInfo", OminiVisionInfo, callback, queue_size=1)
    rospy.Subscriber("/" + str(ROBOT_NAME) + "/ballisholding/BallIsHolding", BallIsHolding, holdingballcallback, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
