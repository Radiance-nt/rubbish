#!/home/radiance/miniconda3/envs/ros/bin/python
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from wpb_home_behaviors.msg import Coord
import math

_cv_bridge = CvBridge()

depth = None
alpha=3

def on_EVENT_LBUTTONDOWN(event, x, y, flags, param):
    # print(depth[y - alpha:y + alpha, x - alpha:x + alpha].mean())
    if event == cv2.EVENT_LBUTTONDOWN:
        xy = "%d,%d" % (x, y)
        cv2.circle(image, (x, y), 1, (255, 255, 255), thickness = -1)
        cv2.putText(image, xy, (x, y), cv2.FONT_HERSHEY_PLAIN,
                    1.0, (255, 255, 255), thickness = 1)
        print(depth[y - alpha:y + alpha, x - alpha:x + alpha].mean())


def show(image_msg):
    global depth,image
    image = _cv_bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
    depth = cv2.resize(image, (106, 128))
    cv2.imshow('color', image)
    cv2.setMouseCallback("color", on_EVENT_LBUTTONDOWN)
    cv2.waitKey(1)


if __name__ == '__main__':
    rospy.init_node('Get_depth', anonymous=True)
    rate = rospy.Rate(10)

    human_point_pub = rospy.Publisher('/Human_pose', Coord, queue_size=100)
    # rospy.Subscriber("/kinect2/sd/image_color_rect", Image, getPositionCallback)
    rospy.Subscriber("/kinect2/sd/image_color_rect", Image, show)
    rospy.spin()
