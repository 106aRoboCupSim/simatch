#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelState
import numpy as np
from nubot_common.msg import ActionCmd, VelCmd, OminiVisionInfo, BallInfo, ObstaclesInfo, RobotInfo

pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
rospy.init_node('reset_ball', anonymous=True)


def callback(data):
    b = data.ballinfo
    ball_x = b.pos.x
    ball_y = b.pos.y

    if ball_x >= 1100:
        resetBall = ModelState()
        resetBall.model_name = 'football'
        resetBall.pose.position.x = 1050
        resetBall.pose.position.y = ball_y
        resetBall.pose.position.z = 0.0
        resetBall.pose.orientation.x = 0.0
        resetBall.pose.orientation.y = 0.0
        resetBall.pose.orientation.z = 0.0
        resetBall.pose.orientation.w = 0.0
        pub.publish()

    if ball_y >= 700:
        resetBall = ModelState()
        resetBall.model_name = 'football'
        resetBall.pose.position.x = ball_x
        resetBall.pose.position.y = -650
        resetBall.pose.position.z = 0.0
        resetBall.pose.orientation.x = 0.0
        resetBall.pose.orientation.y = 0.0
        resetBall.pose.orientation.z = 0.0
        resetBall.pose.orientation.w = 0.0
        pub.publish(resetBall)

    if ball_x <= -1100:
        resetBall = ModelState()
        resetBall.model_name = 'football'
        resetBall.pose.position.x = -1050
        resetBall.pose.position.y = ball_y
        resetBall.pose.position.z = 0.0
        resetBall.pose.orientation.x = 0.0
        resetBall.pose.orientation.y = 0.0
        resetBall.pose.orientation.z = 0.0
        resetBall.pose.orientation.w = 0.0
        pub.publish(resetBall)

    if ball_y <= -700:
        resetBall = ModelState()
        resetBall.model_name = 'football'
        resetBall.pose.position.x = ball_x
        resetBall.pose.position.y = -650
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
