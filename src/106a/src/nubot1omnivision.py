#!/usr/bin/env python
import rospy
from nubot_common.msg import OminiVisionInfo, BallInfo, ObstaclesInfo, RobotInfo

def callback(data):
    

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/NuBot1/omnivision/OmniVisionInfo", OminiVisionInfo, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()