#!/usr/bin/env python
import rospy
import numpy as np
from nubot_common.msg import ActionCmd, VelCmd, OminiVisionInfo, BallInfo, ObstaclesInfo, RobotInfo
from RRT import RRT_closest, RRT
pub = rospy.Publisher('/NuBot1/nubotcontrol/actioncmd', ActionCmd, queue_size=1)
rospy.init_node('pubsub', anonymous=True)
hertz = 100
rate = rospy.Rate(hertz) # 10hz



def P_controller(error_x, error_y, error_tht, d_xdes, d_ydes, d_thtdes):
    control = VelCmd()

    timeConst_x = 0.1
    timeConst_y = 0.1
    timeConst_tht = 0.1

    control.Vx =  -(1/timeConst_x)*error_x + d_xdes
    control.Vy =  -(1/timeConst_y)*error_y + d_ydes
    control.w  =  -(1/timeConst_tht)*error_tht + d_thtdes

    return control

def target_global_to_robot_coords(t_global_x, t_global_y, r_global_x, r_global_y, theta):
    c = np.cos(-theta)
    s = np.sin(-theta)
    t_robot_x = -(c * t_global_x) + (s * t_global_y) + (t_global_x - r_global_x)
    t_robot_y =  (s * t_global_x) + (s * t_global_y) + (t_global_y - r_global_y)
    
    return [t_robot_x, t_robot_y]



def callback(data):
    b = data.ballinfo
    ball_x = b.pos.x
    ball_y = b.pos.y

    r = data.robotinfo[0]
    robot_x = r.pos.x
    robot_y = r.pos.y
    theta = r.heading.theta
    rrt = RRT_closest(start=[robot_x, robot_y], goal=[ball_x, ball_y], rand_area=[-1100, 1100], obstacle_list=[(-500, 0, 100)], max_iter=1)
    #rrt = RRT_closest(start=[robot_x, robot_y], goal=[ball_x, ball_y], rand_area=[-1100, 1100], obstacle_list=[], max_iter=10)

    path = rrt.planning(animation=False)

    # print('robot pos:')
    # print(robot_x)
    # print(robot_y)
    # print('ball_pos')
    # print(ball_x)
    # print(ball_y)
    # print('path:')
    # print(path)
    current_pos = [robot_x, robot_y]
    target=path[0]
    #target = [ball_x, ball_y]
    print(path)
    # print(robot_x)
    # print(robot_y)
    # print(ball_x)
    # print(ball_y)
    # print(path)
    #target = [ball_x, ball_y]
    #print(target)
    target = target_global_to_robot_coords(target[0], target[1], robot_x, robot_y, theta)
    print(target)
    action = ActionCmd()
    action.target.x = target[0]
    action.target.y = target[1]
    action.maxvel = 300
    action.handle_enable = 1

    action.target_ori = 0
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
    rate.sleep()
    
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
    rospy.Subscriber("/NuBot1/omnivision/OmniVisionInfo", OminiVisionInfo, callback, queue_size=1)

    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
