#!/usr/bin/env python
import rospy
import time
import numpy as np
import math
from nubot_common.msg import ActionCmd, VelCmd, OminiVisionInfo, BallInfo, ObstaclesInfo, RobotInfo
from RRT import RRT_closest, RRT
import matplotlib.pyplot as plt
pub = rospy.Publisher('/rival1/nubotcontrol/actioncmd', ActionCmd, queue_size=1)
rospy.init_node('pubsub', anonymous=True)
# hertz = 2
# rate = rospy.Rate(hertz) # 10hz
# rate2 = rospy.Rate(3)
# debugging tools
targets_generated_x = []
targets_generated_y = []
robot_position_x = []
robot_position_y = []

def P_controller(error_x, error_y, error_tht, d_xdes, d_ydes, d_thtdes):
    control = VelCmd()

    timeConst_x = 0.1
    timeConst_y = 0.1
    timeConst_tht = 0.1

    control.Vx =  -(1/timeConst_x)*error_x + d_xdes
    control.Vy =  -(1/timeConst_y)*error_y + d_ydes
    control.w  =  -(1/timeConst_tht)*error_tht + d_thtdes

    return control

def rrt_simplified(target_pos, current_pos, obstacles, target_distance, generate_num=6):
    if clear_path(target_pos, current_pos, obstacles):
        print('path clear between robot and ball')
        return target_pos
    best_target = current_pos
    #heading_options = np.random.random((generate_num,)) * np.pi * 2
    heading_options = np.linspace(0.0, 2*np.pi, num=generate_num)
    for i in heading_options:
        #generate target_point at desired angle, distance
        potential_target = current_pos + np.array([target_distance*np.cos(i), target_distance*np.sin(i)])
        #see if there is a clear path between target point and current point
        #check to see if path gets closest to goal
        if clear_path(potential_target, current_pos, obstacles):
          #print('clear path between potential and current')
            if np.linalg.norm(potential_target - target_pos) < np.linalg.norm(best_target - target_pos):
                print('best target changed')
                best_target = potential_target
    return best_target

def clear_path(target_pos, current_pos, obstacles):
    AB = target_pos - current_pos
    # print('AB', AB)
    for o in obstacles:
        AC = o[:2] - current_pos
        # print('AC', AC)
        AD = AB * np.dot(AC, AB) / np.dot(AB, AB)
        # print('AD', AD)
        D = current_pos + AD
        # print('D', D)
        # print('distance between D and obstacle center', np.linalg.norm(D - o[:2]))
        if np.linalg.norm(D - o[:2]) <= o[2]:
            return False
    return True




def target_global_to_robot_coords(t_global_x, t_global_y, r_global_x, r_global_y, theta):
    c = np.cos(theta)
    s = np.sin(theta)
    x = t_global_x - r_global_x
    y = t_global_y - r_global_y
    t_robot_x = c*x + s*y
    t_robot_y = -s*x + c*y
    # #t_robot_x =  (c * t_glo
    # c = np.cos(theta)
    # s = np.sin(theta)
    # t_robot_x =  (c * t_global_x) - (s * t_global_y) + (t_global_x - r_global_x)
    # t_robot_y =  (s * t_global_x) + (c * t_global_y) + (t_global_y - r_global_y)
    
    return [t_robot_x, t_robot_y]

def plot_circle(x, y, size, color="-b"):  # pragma: no cover
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
        plt.plot(xl, yl, color)



def callback(data):
    b = data.ballinfo
    ball_pos = np.array([b.pos.x, b.pos.y])

    r = data.robotinfo[0]
    robot_pos = np.array([r.pos.x, r.pos.y])
    theta = r.heading.theta
    #rrt = RRT_closest(start=[robot_x, robot_y], goal=[ball_x, ball_y], rand_area=[-1100, 1100], obstacle_list=[(-500, 0, 100)], max_iter=5)
    #rrt = RRT_closest(start=[robot_x, robot_y], goal=[ball_x, ball_y], rand_area=[-1100, 1100], obstacle_list=[], max_iter=10)

    #path = rrt.planning(animation=False)

    # print('robot pos:')
    # print(robot_x)
    # print(robot_y)
    # print('ball_pos')
    # print(ball_x)
    # print(ball_y)
    # print('path:')
    # print(path)
    #obstacle_list = np.array([[-0, -400, 200], [0, 400, 200], [-500, 0, 200], [-500, -100, 100]])
    obstacles = data.obstacleinfo
    obstacle_list = np.empty((0,3), float)
    #print(obstacles.pos)
    for p in obstacles.pos:
        obstacle_list = np.concatenate((obstacle_list, np.array([[p.x, p.y, 100]])))
        #np.append(obstacle_list, [[p.x, p.y, 100]])

    target = rrt_simplified(ball_pos, robot_pos, obstacle_list, 100, 400)
    robot_position_x.append(robot_pos[0])
    robot_position_y.append(robot_pos[1])
    targets_generated_x.append(target[0])
    targets_generated_y.append(target[1])
    #target = [-400, 500]
    # target=path[0]
    #target = [ball_x, ball_y]
    print('ball_pos', ball_pos)
    print('target_pos', target)
    print('robot_pos', robot_pos)
    # print(robot_x)
    # print(robot_y)
    # print(ball_x)
    # print(ball_y)
    # print(path)
    #target = [ball_x, ball_y]
    #print(target)
    target = target_global_to_robot_coords(target[0], target[1], robot_pos[0], robot_pos[1], theta)
    print(target)
    
    #print(target)
    action = ActionCmd()
    action.target.x = target[0]
    action.target.y = target[1]
    action.maxvel = 300
    action.handle_enable = 1

    #action.target_ori = 0
    # action.target.x = -target[0]
    # action.target.y = -target[1]
    # action.maxvel = 100
    # action.target_vel.x = (target[0] - current_pos[0]) * hertz
    # action.target_vel.y = (target[1] - current_pos[1]) * hertz
    # vel = VelCmd()
    # vel.Vx = (target[0] - current_pos[0]) * hertz
    # vel.Vy = (target[1] - current_pos[1]) * hertz
    # vel.w = 0

    pub.publish(action)
    time.sleep(0.5)
    #rate2.sleep()
    action  = ActionCmd()
    action.maxvel = 0

    pub.publish(action)
    print(len(targets_generated_x))
    if len(targets_generated_x) > 60:
        fig, ax = plt.subplots()
        ax.scatter(targets_generated_x, targets_generated_y)
        ax.scatter(robot_position_x, robot_position_y)
        for i in range(len(targets_generated_x)):
            ax.annotate(i, (targets_generated_x[i], targets_generated_y[i]))
            ax.annotate(i, (robot_position_x[i], robot_position_y[i]))
        for o in obstacle_list:
            plot_circle(o[0], o[1], o[2])
        #print(targets_generated)
        plt.show()
        time.sleep(100)
    time.sleep(0.5)
    #rate.sleep()
    
    #c = P_controller(target[0] - prev_target[0], target[1] - prev_target[1], 0, target[0], target[1], 0)
    #c = P_controller(prev_target[0] - target[0], prev_target[1] - target[1], 0, target[0], target[1], 0)
    #c = P_controller(prev_target[0] - target[0], prev_target[1] - target[1], 0, 0, 0, 0)
    #pub.publish(c)
    #rate.sleep()
    # print('robot pos:')
    # print(robot_x)
    # print(robot_y)
    # print('ball_pos')
    # print(ball_x)
    # print(ball_y)
    # list.reverse(path)
    # print('path:')
    # print(path)
    # print(np.shape(path))
    # for target in path:
    #     c = P_controller(prev_target[0] - prev_target[0], prev_target[1] - target[1], 0, target[0], target[1], 0)
    #     pub.publish(c)
    #     prev_target = target
    #     rate.sleep()
    


def listener():
    rospy.Subscriber("/rival1/omnivision/OmniVisionInfo", OminiVisionInfo, callback, queue_size=1)

    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
