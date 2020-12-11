#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelState
import numpy as np
from nubot_common.msg import ActionCmd, VelCmd, OminiVisionInfo, BallInfo, ObstaclesInfo, RobotInfo
import time

pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
rospy.init_node('ball_manager', anonymous=True)


def callback(data):
    # r = data.robotinfo
    # print(len(r))
    b = data.ballinfo
    ball_x = b.pos.x/100
    ball_y = b.pos.y/100
    # print('ball_x: ' +str(ball_x))
    # print('ball_y: ' +str(ball_y))
    if abs(ball_x) >= 11 or abs(ball_y) >= 7:
        #time.sleep(1.5)
        resetBall = ModelState()
        resetBall.model_name = 'football'
        resetBall.pose.position.x = 0.0
        resetBall.pose.position.y = 0.0
        resetBall.pose.position.z = 0.0
        resetBall.pose.orientation.x = 0.0
        resetBall.pose.orientation.y = 0.0
        resetBall.pose.orientation.z = 0.0
        resetBall.pose.orientation.w = 0.0
        pub.publish(resetBall)

        reset_nubot = ModelState()
        reset_nubot.model_name = 'NuBot1'
        reset_nubot.pose.position.x = -10.5
        reset_nubot.pose.position.y = 0
        reset_nubot.pose.position.z = 0.0
        reset_nubot.pose.orientation.x = 0.0
        reset_nubot.pose.orientation.y = 0.0
        reset_nubot.pose.orientation.z = 0.0
        reset_nubot.pose.orientation.w = 0.0
        pub.publish(reset_nubot)

        # for i in range(4):
        #     reset_rival = ModelState()
        #     reset_rival.model_name = 'rival' + str(i + 1)
        #     reset_rival.pose.position.x = -6 + 2 * i
        #     reset_rival.pose.position.y = 0
        #     reset_rival.pose.position.z = 0.0
        #     reset_rival.pose.orientation.x = 0.0
        #     reset_rival.pose.orientation.y = 0.0
        #     reset_rival.pose.orientation.z = 0.0
        #     reset_rival.pose.orientation.w = 0.0
        #     pub.publish(reset_rival)


def listener():
    rospy.Subscriber("/NuBot1/omnivision/OmniVisionInfo", OminiVisionInfo, callback, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
