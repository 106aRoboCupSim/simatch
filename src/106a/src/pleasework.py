#!/usr/bin/env python
import rospy
from nubot_common.msg import ActionCmd

def talker():
    pub = rospy.Publisher('/NuBot1/nubotcontrol/actioncmd', ActionCmd, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    action = ActionCmd()
    action.target.x = 10000.0
    action.target.y = 0.0
    action.maxvel = 3000

    while not rospy.is_shutdown():
        pub.publish(action)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass