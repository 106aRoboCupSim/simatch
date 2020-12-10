#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelState

def talker():
    pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
    rospy.init_node('reset', anonymous=True)


    resetNuBot1 = ModelState()
    resetNuBot1.model_name = 'NuBot1'
    resetNuBot1.pose.position.x = -11.0
    resetNuBot1.pose.position.y = 0.0
    resetNuBot1.pose.position.z = 0.0
    resetNuBot1.pose.orientation.x = 0.0
    resetNuBot1.pose.orientation.y = 0.0
    resetNuBot1.pose.orientation.z = -1.0
    resetNuBot1.pose.orientation.w = 0.0

    resetBall = ModelState()
    resetBall.model_name = 'football'
    resetBall.pose.position.x = 0.0
    resetBall.pose.position.y = 0.0
    resetBall.pose.position.z = 0.0
    resetBall.pose.orientation.x = 0.0
    resetBall.pose.orientation.y = 0.0
    resetBall.pose.orientation.z = 0.0
    resetBall.pose.orientation.w = 0.0

    pub.publish(resetNuBot1)
    pub.publish(resetBall)
    print('reset')
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass