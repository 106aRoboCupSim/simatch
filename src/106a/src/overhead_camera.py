#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
bridge = CvBridge()
from sensor_msgs.msg import Image

params = cv2.SimpleBlobDetector_Params()

# Change thresholds
params.minThreshold = 0
params.maxThreshold = 2000

# Filter by Area.
params.filterByArea = True
params.minArea = 100
params.maxArea = 20000

# Filter by Circularity
params.filterByCircularity = True
params.minCircularity = 0.0

# Filter by Convexity
params.filterByConvexity = True
params.minConvexity = 0.0

# Filter by Inertia
params.filterByInertia = True
params.minInertiaRatio = 0.0


detector = cv2.SimpleBlobDetector_create(params)

#For masking out the field
low_green = np.array([45, 75, 0])
high_green = np.array([140, 180, 90])
low_white = np.array([200, 200, 200])
high_white = np.array([255, 255, 255])

def callback(data):
    cv_image = bridge.imgmsg_to_cv2(data)[:,150:650]

    #Mask out field and lines
    mask = cv2.inRange(cv_image, low_green, high_green)
    mask2 = cv2.inRange(cv_image, low_white, high_white)
    mask = 255-(mask + mask2)
    cv_image = cv2.bitwise_and(cv_image, cv_image, mask=mask)
    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    cv_image = cv2.blur(cv_image, (10,10))
    cv_image = cv2.equalizeHist(cv_image)

    #Run blob detection
    keypoints = detector.detect(mask)
    im_with_keypoints = cv2.cvtColor(cv2.drawKeypoints(mask, keypoints, np.array([]), (255,69,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS), cv2.COLOR_BGR2GRAY)
    cv_image = cv2.bitwise_and(cv_image,im_with_keypoints,mask = mask)


def listener():
    rospy.init_node('overhead_camera', anonymous=False)
    rospy.Subscriber("/rrbot/camera1/image_raw", Image, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()