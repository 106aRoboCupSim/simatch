#!/usr/bin/env python

import rospy
from nubot_common.msg import VelCmd

def talker():
    pub = rospy.Publisher('/NuBot1/nubotcontrol/velcmd', VelCmd, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    v = VelCmd()
    v.Vx = 10
    v.Vy = 10
    v.w = 0
    while not rospy.is_shutdown():
        pub.publish(v)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass