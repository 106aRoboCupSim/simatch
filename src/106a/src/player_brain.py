#!/usr/bin/env python
import rospy
import sys
import time
import numpy as np
from numpy import linalg as LA
import random
from realtimepseudoAstar import plan
from globaltorobotcoords import transform
from std_msgs.msg import String
from nubot_common.msg import ActionCmd, VelCmd, OminiVisionInfo, BallInfo, ObstaclesInfo, RobotInfo, BallIsHolding

ROBOT_NAME = 'NuBot' + str(sys.argv[1])
if str(sys.argv[2]) == '1':
    ROBOT_NAME = 'rival' + str(sys.argv[1])
opponent_goal = np.array([1100.0, 0.0])
isdribble = 0
ball_handler1 = (0, "NuBot1") 
ball_handler2 = (0, "NuBot2") 
ball_handler3 = (0, "NuBot3") 
ball_handler4 = (0, "rival1") 
ball_handler5 = (0, "rival2")
ball_handler6 = (0, "rival3")
ball_handler_current = "nobody"
shoot_range = 500


#Bare minimum is 50
obstacle_radius = 300

if str(sys.argv[1]) == '2':
    my_id = 1
    mate_id = 2
if str(sys.argv[1]) == '3':
    my_id = 2
    mate_id = 1

# For plotting
# import math
# import matplotlib.pyplot as plt

# Initialize publisher and rate
pub = rospy.Publisher('/' + str(ROBOT_NAME)+'/nubotcontrol/actioncmd', ActionCmd, queue_size=1)
rospy.init_node(str(ROBOT_NAME) + '_brain', anonymous=False)
hertz = 10
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

    def in_range(pos_1, pos_2, dist=300):
        val = np.linalg.norm(pos_1 - pos_2)
        return val < dist

    def exists_clear_path(goal_pos, current_pos, obstacles):
        AB = goal_pos - current_pos
        for o in obstacles:
            AC = o[:2] - current_pos
            AD = AB * np.dot(AC, AB) / np.dot(AB, AB)
            D = current_pos + AD
            if np.linalg.norm(D - o[:2]) <= o[2]:
                return False
        return True

    action = ActionCmd()

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
    r = data.robotinfo[my_id]
    my_pos = np.array([r.pos.x, r.pos.y])
    my_theta = r.heading.theta

    #Get teammate position and heading in global frame
    r_2 = data.robotinfo[mate_id]
    mate_pos = np.array([r_2.pos.x, r_2.pos.y])
    mate_theta = r.heading.theta

    #Get obstacle positions in global frame
    obstacles = data.obstacleinfo
    obstacle_list = np.empty((0,3), float)
    for p in obstacles.pos:
        obstacle_list = np.concatenate((obstacle_list, np.array([[p.x, p.y, 75]])))

    # In range to pass
    passing = False
    if isdribble and in_range(my_pos, mate_pos, dist=500):
        dist_to_teammate = LA.norm(my_pos - mate_pos)
        dist_to_goal = LA.norm(my_pos - opponent_goal)
        ang_to_teammate = np.arctan2(mate_pos[1] - my_pos[1], mate_pos[0] - my_pos[0]) - my_theta
        obstructed_goal = not exists_clear_path(opponent_goal, my_pos, obstacle_list)
        obstructed_mate = not exists_clear_path(mate_pos, my_pos, obstacle_list)
        if ((dist_to_teammate < dist_to_goal) \
            or (obstructed_goal and not obstructed_mate)) \
            and (dist_to_teammate > 200) \
            and (ang_to_teammate-np.pi/30 < my_theta and my_theta < ang_to_teammate+np.pi/30):

            target = my_pos
            thetaDes = ang_to_teammate
            action.maxvel = 250
            action.shootPos = 1
            action.strength = dist_to_teammate / 50
            passing = True

    def has_ball_priority(my_pos, mate_pos):
        my_dist_to_ball = LA.norm(my_pos - ball_pos)
        mate_dist_to_ball = LA.norm(mate_pos - ball_pos)
        return my_dist_to_ball < mate_dist_to_ball

    def team_has_ball():
        return ball_handler_current[0:5] == ROBOT_NAME[0:5]
    
    # WINGMAN: GET OPEN
    if not passing and not has_ball_priority(my_pos, mate_pos) and team_has_ball():
        target = [mate_pos[0]+(opponent_goal[0]-mate_pos[0])/2, 100-mate_pos[1]/2]
        thetaDes = np.arctan2(mate_pos[1] - my_pos[1], mate_pos[0] - my_pos[0]) - my_theta
        target = transform(target[0], target[1], my_pos[0], my_pos[1], my_theta)
        action.maxvel = 300
    elif not passing and not has_ball_priority(my_pos, mate_pos):
        target = plan(ball_pos, my_pos, obstacle_list, obstacle_radius, 400)
        thetaDes = np.arctan2(target[1] - my_pos[1], target[0] - my_pos[0]) - my_theta
        target = transform(target[0], target[1], my_pos[0], my_pos[1], my_theta)
        action.maxvel = 300


    # AGGRESSOR: GET BALL
    if not passing and has_ball_priority(my_pos, mate_pos):
        action.maxvel = 250
        #print(obstacle_list)
        #print(r.isdribble)
        target = plan(ball_pos, my_pos, obstacle_list, obstacle_radius, 400)
        thetaDes = np.arctan2(target[1] - my_pos[1], target[0] - my_pos[0]) - my_theta
        #print(isdribble)
        clear_shot = exists_clear_path(opponent_goal, my_pos, obstacle_list)

        if isdribble and np.linalg.norm(opponent_goal - my_pos) > shoot_range:
            target = plan(opponent_goal, my_pos, obstacle_list, obstacle_radius, 400)
            thetaDes = np.arctan2(opponent_goal[1] - my_pos[1], opponent_goal[0] - my_pos[0]) - my_theta
        elif clear_shot and isdribble and np.linalg.norm(opponent_goal - my_pos) < shoot_range:
            thetaDes = np.arctan2(opponent_goal[1] - my_pos[1], opponent_goal[0] - my_pos[0]) - my_theta
            target = my_pos
            action.shootPos = 1
            action.strength = 200
        elif isdribble and np.linalg.norm(opponent_goal - my_pos) < shoot_range:
            dist_to_teammate = LA.norm(my_pos - mate_pos)
            dist_to_goal = LA.norm(my_pos - opponent_goal)
            ang_to_teammate = np.arctan2(mate_pos[1] - my_pos[1], mate_pos[0] - my_pos[0]) - my_theta
            obstructed_goal = not exists_clear_path(opponent_goal, my_pos, obstacle_list)
            obstructed_mate = not exists_clear_path(mate_pos, my_pos, obstacle_list)
            target = my_pos
            thetaDes = ang_to_teammate
            action.maxvel = 250
            action.shootPos = 1
            action.strength = dist_to_teammate / 50
            passing = True
        
        target = transform(target[0], target[1], my_pos[0], my_pos[1], my_theta)

    #Generate target position and heading in global frame from real-time psuedo A-star path planning algorithm
    # target = plan(ball_pos, my_pos, obstacle_list, 100, 400)
    # thetaDes = np.arctan2(target[1] - my_pos[1], target[0] - my_pos[0])

    # For plotting
    # my_position_x.append(my_pos[0])
    # my_position_y.append(my_pos[1])
    # targets_generated_x.append(target[0])
    # targets_generated_y.append(target[1])

    #Convert target from global coordinate frame to robot coordinate frame for use by hwcontroller


    #Generate ActionCmd() and publish to hwcontroller
    #action = ActionCmd()
    action.target.x = target[0]
    action.target.y = target[1]
    action.maxw = 300
    action.handle_enable = 1
    action.target_ori = thetaDes
    pub.publish(action)
    rate.sleep()


    #   # For plotting path and path plan of robot, after 100 pathing iterations
    # if len(targets_generated_x) > 100:
    #     fig, ax = plt.subplots()
    #     ax.scatter(targets_generated_x, targets_generated_y)
    #     ax.scatter(my_position_x, my_position_y)
    #     for i in range(len(targets_generated_x)):
    #         ax.annotate(i, (targets_generated_x[i], targets_generated_y[i]))
    #         ax.annotate(i, (my_position_x[i], my_position_y[i]))
    #     for o in obstacle_list:
    #         plot_circle(o[0], o[1], o[2])
    #     #print(targets_generated)
    #     plt.show()
    #     time.sleep(100)

def holdingballcallback(data):
    global isdribble
    isdribble = data.BallIsHolding
    
def update_holder1(data):
    global ball_handler1
    ball_handler1 = (data.BallIsHolding, "NuBot1")
    update_holder_consolidate()

def update_holder2(data):
    global ball_handler2
    ball_handler2 = (data.BallIsHolding, "NuBot2")
    update_holder_consolidate()

def update_holder3(data):
    global ball_handler3
    ball_handler3 = (data.BallIsHolding, "NuBot3")
    update_holder_consolidate()

def update_holder4(data):
    global ball_handler4
    ball_handler4 = (data.BallIsHolding, "rival1")
    update_holder_consolidate()

def update_holder5(data):
    global ball_handler5
    ball_handler5 = (data.BallIsHolding, "rival2")
    update_holder_consolidate()

def update_holder6(data):
    global ball_handler6
    ball_handler6 = (data.BallIsHolding, "rival3")
    update_holder_consolidate()

def update_holder_consolidate():
    global ball_handler_current
    l = [ball_handler1, ball_handler2, ball_handler3, ball_handler4, ball_handler5, ball_handler6]
    for i in range(6):
        a = l[i]
        data, name = a
        if int(data) == 1:
            ball_handler_current = name
            return
    ball_handler_current = "nobody"
    


def listener():
    rospy.Subscriber("/" + str(ROBOT_NAME) + "/omnivision/OmniVisionInfo", OminiVisionInfo, callback, queue_size=1)
    rospy.Subscriber("/" + str(ROBOT_NAME) + "/ballisholding/BallIsHolding", BallIsHolding, holdingballcallback, queue_size=1)
    rospy.Subscriber("/NuBot1/ballisholding/BallIsHolding", BallIsHolding, update_holder1, queue_size=1)
    rospy.Subscriber("/NuBot2/ballisholding/BallIsHolding", BallIsHolding, update_holder2, queue_size=1)
    rospy.Subscriber("/NuBot3/ballisholding/BallIsHolding", BallIsHolding, update_holder3, queue_size=1)
    rospy.Subscriber("/rival1/ballisholding/BallIsHolding", BallIsHolding, update_holder4, queue_size=1)
    rospy.Subscriber("/rival2/ballisholding/BallIsHolding", BallIsHolding, update_holder5, queue_size=1)
    rospy.Subscriber("/rival3/ballisholding/BallIsHolding", BallIsHolding, update_holder6, queue_size=1)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
