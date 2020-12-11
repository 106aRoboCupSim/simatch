#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelState
import numpy as np
from nubot_common.msg import ActionCmd, VelCmd, OminiVisionInfo, BallInfo, ObstaclesInfo, RobotInfo
import time

pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
rospy.init_node('ball_manager', anonymous=True)


def callback(data):
    # r = data.robotinfo
    # print(len(r))
    b = data.ballinfo
    ball_x = b.pos.x/100
    ball_y = b.pos.y/100
    # print('ball_x: ' +str(ball_x))
    # print('ball_y: ' +str(ball_y))
    sleeptime = 0
    if ball_x >= 11 and abs(ball_y) >= 1.25:
        time.sleep(sleeptime)
        resetBall = ModelState()
        resetBall.model_name = 'football'
        resetBall.pose.position.x = 10
        resetBall.pose.position.y = ball_y
        resetBall.pose.position.z = 0.0
        resetBall.pose.orientation.x = 0.0
        resetBall.pose.orientation.y = 0.0
        resetBall.pose.orientation.z = 0.0
        resetBall.pose.orientation.w = 0.0
        pub.publish(resetBall)

    if ball_x >= 11 and abs(ball_y) <= 1.25:
        time.sleep(sleeptime)
        black_goal = ModelState()
        black_goal.model_name = 'football'
        black_goal.pose.position.x = 0.0
        black_goal.pose.position.y = 0.0
        black_goal.pose.position.z = 0.0
        black_goal.pose.orientation.x = 0.0
        black_goal.pose.orientation.y = 0.0
        black_goal.pose.orientation.z = 0.0
        black_goal.pose.orientation.w = 0.0
        pub.publish(black_goal)
        for i in [1,2]:
            reset_nubot = ModelState()
            reset_nubot.model_name = 'NuBot'+str(i)
            reset_rival = ModelState()
            reset_rival.model_name = 'rival'+str(i)
            if i == 1:
                reset_nubot.pose.position.x = -10.5
                reset_nubot.pose.position.y = 0
                reset_nubot.pose.position.z = 0
                reset_nubot.pose.orientation.x = 0.0
                reset_nubot.pose.orientation.y = 0.0
                reset_nubot.pose.orientation.z = 0.0
                reset_nubot.pose.orientation.w = 0.0

                reset_rival.pose.position.x = 10.5
                reset_rival.pose.position.y = 0
                reset_rival.pose.position.z = 0
                reset_rival.pose.orientation.x = 0.0
                reset_rival.pose.orientation.y = 0.0
                reset_rival.pose.orientation.z = 0.0
                reset_rival.pose.orientation.w = np.pi
                pub.publish(reset_nubot)
                pub.publish(reset_rival)
            if i == 2:
                reset_nubot.pose.position.x = -2
                reset_nubot.pose.position.y = 1.00
                reset_nubot.pose.position.z = 0
                reset_nubot.pose.orientation.x = 0.0
                reset_nubot.pose.orientation.y = 0.0
                reset_nubot.pose.orientation.z = 0.0
                reset_nubot.pose.orientation.w = 0

                reset_rival.pose.position.x = 2.0
                reset_rival.pose.position.y = -1.00
                reset_rival.pose.position.z = 0
                reset_rival.pose.orientation.x = 0.0
                reset_rival.pose.orientation.y = 0.0
                reset_rival.pose.orientation.z = 0.0
                reset_rival.pose.orientation.w = np.pi
                pub.publish(reset_nubot)
                pub.publish(reset_rival)


    if ball_y >= 7:
        time.sleep(sleeptime)
        resetBall = ModelState()
        resetBall.model_name = 'football'
        resetBall.pose.position.x = ball_x
        resetBall.pose.position.y = 6
        resetBall.pose.position.z = 0.0
        resetBall.pose.orientation.x = 0.0
        resetBall.pose.orientation.y = 0.0
        resetBall.pose.orientation.z = 0.0
        resetBall.pose.orientation.w = 0.0
        pub.publish(resetBall)

    if ball_x <= -11 and abs(ball_y) >= 1.25:
        time.sleep(sleeptime)
        resetBall = ModelState()
        resetBall.model_name = 'football'
        resetBall.pose.position.x = -10
        resetBall.pose.position.y = ball_y
        resetBall.pose.position.z = 0.0
        resetBall.pose.orientation.x = 0.0
        resetBall.pose.orientation.y = 0.0
        resetBall.pose.orientation.z = 0.0
        resetBall.pose.orientation.w = 0.0
        pub.publish(resetBall)

    if ball_x <= -11 and abs(ball_y) <= 1.25:
        time.sleep(sleeptime)
        red_goal = ModelState()
        red_goal.model_name = 'football'
        red_goal.pose.position.x = 0.0
        red_goal.pose.position.y = 0.0
        red_goal.pose.position.z = 0.0
        red_goal.pose.orientation.x = 0.0
        red_goal.pose.orientation.y = 0.0
        red_goal.pose.orientation.z = 0.0
        red_goal.pose.orientation.w = 0.0
        pub.publish(red_goal)

        for i in [1,2]:
            reset_nubot = ModelState()
            reset_nubot.model_name = 'NuBot'+str(i)
            reset_rival = ModelState()
            reset_rival.model_name = 'rival'+str(i)
            if i == 1:
                reset_nubot.pose.position.x = -10.5
                reset_nubot.pose.position.y = 0
                reset_nubot.pose.position.z = 0
                reset_nubot.pose.orientation.x = 0.0
                reset_nubot.pose.orientation.y = 0.0
                reset_nubot.pose.orientation.z = 0.0
                reset_nubot.pose.orientation.w = 0.0

                reset_rival.pose.position.x = 10.5
                reset_rival.pose.position.y = 0
                reset_rival.pose.position.z = 0
                reset_rival.pose.orientation.x = 0.0
                reset_rival.pose.orientation.y = 0.0
                reset_rival.pose.orientation.z = 0.0
                reset_rival.pose.orientation.w = np.pi
                pub.publish(reset_nubot)
                pub.publish(reset_rival)
            if i == 2:
                reset_nubot.pose.position.x = -2
                reset_nubot.pose.position.y = 1.00
                reset_nubot.pose.position.z = 0
                reset_nubot.pose.orientation.x = 0.0
                reset_nubot.pose.orientation.y = 0.0
                reset_nubot.pose.orientation.z = 0.0
                reset_nubot.pose.orientation.w = 0

                reset_rival.pose.position.x = 2.0
                reset_rival.pose.position.y = -1.00
                reset_rival.pose.position.z = 0
                reset_rival.pose.orientation.x = 0.0
                reset_rival.pose.orientation.y = 0.0
                reset_rival.pose.orientation.z = 0.0
                reset_rival.pose.orientation.w = np.pi
                pub.publish(reset_nubot)
                pub.publish(reset_rival)

    if ball_y <= -7:
        time.sleep(sleeptime)
        resetBall = ModelState()
        resetBall.model_name = 'football'
        resetBall.pose.position.x = ball_x
        resetBall.pose.position.y = -6
        resetBall.pose.position.z = 0.0
        resetBall.pose.orientation.x = 0.0
        resetBall.pose.orientation.y = 0.0
        resetBall.pose.orientation.z = 0.0
        resetBall.pose.orientation.w = 0.0
        pub.publish(resetBall)

    # rate.sleep()

def listener():
    rospy.Subscriber("/NuBot1/omnivision/OmniVisionInfo", OminiVisionInfo, callback, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
