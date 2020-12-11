#!/usr/bin/env python
import rospy
from nubot_common.msg import OminiVisionInfo, BallInfo, ObstaclesInfo, RobotInfo

def callback(data):
    b = data.ballinfo
    o = data.obstacleinfo
    r = data.robotinfo

    # print('start')
    # print(b.ballinfostate)
    # print(b.pos)
    # print(b.real_pos)
    # print(b.velocity)
    # print(b.pos_known)
    # print(b.velocity_known)
    # print('end')
    for robot in r:
        print(robot.targetNum1)
        print(robot.targetNum2)
        print(robot.targetNum3)
        print(robot.targetNum4)
        print(robot.pos)
    


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/NuBot1/omnivision/OmniVisionInfo", OminiVisionInfo, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()