#!/usr/bin/env python
import rospy
import numpy as np
import sys
from nubot_common.msg import ActionCmd, VelCmd, OminiVisionInfo, BallInfo, ObstaclesInfo, RobotInfo

rival_number = sys.argv[1]
print(sys.argv[1])

pub = rospy.Publisher('/' + str(rival_number)+'/nubotcontrol/actioncmd', ActionCmd, queue_size=1)
rospy.init_node('pubsub_for_'+ str(rival_number), anonymous=True)
hertz = 100
rate = rospy.Rate(hertz) # 10hz

def target_global_to_robot_coords(t_global_x, t_global_y, r_global_x, r_global_y, theta):
    c = np.cos(theta)
    s = np.sin(theta)
    t_robot_x = -(c * t_global_x) + (s * t_global_y) + (t_global_x - r_global_x)
    t_robot_y =  (s * t_global_x) + (s * t_global_y) + (t_global_y - r_global_y)
    # t_robot_x = (t_global_x - r_global_x)- (t_global_y - r_global_y)
    # t_robot_y = (t_global_x - r_global_x)+(t_global_y - r_global_y)
    return [t_robot_x, t_robot_y]



def callback(data):
    b = data.ballinfo
    ball_x = b.pos.x
    ball_y = b.pos.y

    r = data.robotinfo[0]
    print(data.robotinfo)
    robot_x = r.pos.x
    robot_y = r.pos.y
    theta = r.heading.theta

    #rrt = RRT_closest(start=[robot_x, robot_y], goal=[ball_x, ball_y], rand_area=[2200, 1400], obstacle_list=[(-1000, -90, 10)], max_iter=3)
    # path = rrt.planning(animation=False)

    current_pos = np.array([robot_x, robot_y])
    target1 = np.array([500,0])
    target2 = np.array([-500, 0])
    center_target = np.array([0,0])
    # print(np.round(current_pos[0]) == target1[0])
    bool = False
    # print(target)
    # print(np.all(np.round(current_pos)==center_target))
    # print((np.all(np.logical_and(np.round(current_pos), center_target))))
    # print(np.all(np.logical_and(np.round(current_pos),target1)))
    # print(bool)
    if np.all(np.round(current_pos)==center_target) or np.all(np.round(current_pos)==target1) and bool == False:
        target = target_global_to_robot_coords(target2[0], target2[1], robot_x, robot_y, theta)
        action1 = ActionCmd()
        action1.target.x = target[0]
        action1.target.y = target[1]
        action1.maxvel = 300
        pub.publish(action1)
        print('action1'+str(action1))
        # target = target2

    if np.all(np.round(current_pos)== target2) and bool == False:
        target = target_global_to_robot_coords(target1[0], target1[1], robot_x, robot_y, theta)
        action2 = ActionCmd()
        action2.target.x = target[0]
        action2.target.y = target[1]
        action2.maxvel = 300
        pub.publish(action2)

    # if bool == True :
    #     target = target_global_to_robot_coords(center_target[0], center_target[1], robot_x, robot_y, theta)
    #     action3 = ActionCmd()
    #     action3.target.x = target[0]
    #     action3.target.y = target[1]
    #     action3.maxvel = 300
    #     pub.publish(action3)
    #     bool = False





    # print(target)
    # target = target_global_to_robot_coords(target[0], target[1], robot_x, robot_y, theta)
    # action = ActionCmd()
    # action.target.x = target[0]
    # action.target.y = target[1]
    # action.maxvel = 300
    #
    # pub.publish(action)
    rate.sleep()

def listener():
    rospy.Subscriber("/" + str(rival_number) + "/omnivision/OmniVisionInfo", OminiVisionInfo, callback, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
