#!/usr/bin/env python
import rospy
import numpy as np
from realtimepseudoAstar import plan
from globaltorobotcoords import transform
from nubot_common.msg import ActionCmd, VelCmd, OminiVisionInfo, BallInfo, ObstaclesInfo, RobotInfo

# For plotting
# import math
# import matplotlib.pyplot as plt

# Initialize publisher and rate
pub = rospy.Publisher('/NuBot1/nubotcontrol/actioncmd', ActionCmd, queue_size=1)
rospy.init_node('pubsub', anonymous=False)
hertz = 10
rate = rospy.Rate(hertz)

# For plotting path and path plan
# targets_generated_x = []
# targets_generated_y = []
# robot_position_x = []
# robot_position_y = []

# def plot_circle(x, y, size, color="-b"):  # pragma: no cover
#         deg = list(range(0, 360, 5))
#         deg.append(0)
#         xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
#         yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
#         plt.plot(xl, yl, color)


def callback(data):
    
    #Get ball position in global frame
    b = data.ballinfo
    ball_pos = np.array([b.pos.x, b.pos.y])

    #Get robot position and heading in global frame
    r = data.robotinfo[0]
    robot_pos = np.array([r.pos.x, r.pos.y])
    theta = r.heading.theta

    #Get obstacle positions in global frame
    obstacles = data.obstacleinfo
    obstacle_list = np.empty((0,3), float)
    for p in obstacles.pos:
        obstacle_list = np.concatenate((obstacle_list, np.array([[p.x, p.y, 100]])))

    #Generate target position and heading in global frame from real-time psuedo A-star path planning algorithm
    target = plan(ball_pos, robot_pos, obstacle_list, 100, 400)
    thetaDes = np.arctan2(target[1] - robot_pos[1], target[0] - robot_pos[0])

    # For plotting
    # robot_position_x.append(robot_pos[0])
    # robot_position_y.append(robot_pos[1])
    # targets_generated_x.append(target[0])
    # targets_generated_y.append(target[1])

    #Convert target from global coordinate frame to robot coordinate frame for use by hwcontroller
    target = transform(target[0], target[1], robot_pos[0], robot_pos[1], theta)
    
    #Generate ActionCmd() and publish to hwcontroller
    action = ActionCmd()
    action.target.x = target[0]
    action.target.y = target[1]
    action.maxvel = 300
    action.handle_enable = 1
    action.target_ori = thetaDes
    pub.publish(action)
    rate.sleep()


    #   # For plotting path and path plan of robot, after 100 pathing iterations
    # if len(targets_generated_x) > 100:
    #     fig, ax = plt.subplots()
    #     ax.scatter(targets_generated_x, targets_generated_y)
    #     ax.scatter(robot_position_x, robot_position_y)
    #     for i in range(len(targets_generated_x)):
    #         ax.annotate(i, (targets_generated_x[i], targets_generated_y[i]))
    #         ax.annotate(i, (robot_position_x[i], robot_position_y[i]))
    #     for o in obstacle_list:
    #         plot_circle(o[0], o[1], o[2])
    #     #print(targets_generated)
    #     plt.show()
    #     time.sleep(100)
    


def listener():
    rospy.Subscriber("/NuBot1/omnivision/OmniVisionInfo", OminiVisionInfo, callback, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
