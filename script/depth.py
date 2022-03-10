#!/home/radiance/miniconda3/envs/ros/bin/python
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from wpb_home_behaviors.msg import Coord
import math

_cv_bridge = CvBridge()
image = None

depth = None
alpha = 3


def on_EVENT_LBUTTONDOWN(event, x, y, flags, param):
    # print(depth[y - alpha:y + alpha, x - alpha:x + alpha].mean())
    global image,depth
    # if event == cv2.EVENT_MOUSEMOVE:
    ii = depth[y - alpha:y + alpha, x - alpha:x + alpha].mean()
    print(ii)
    image = cv2.putText(image, "%.2f" % ii, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    cv2.imshow('depth', image)

    #     xy = "%d,%d" % (x, y)
    #
    #     cv2.putText(img, xy, (x, y), cv2.FONT_HERSHEY_PLAIN,
    #                 1.0, (0, 0, 0), thickness=1)
    #     cv2.imshow("image", img)


def show(image_msg):
    global  image,depth
    depth = _cv_bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
    _, image = cv2.threshold(
        depth, 4000, 4000, cv2.THRESH_TRUNC)
    # image = cv2.resize(image, (106, 128))
    image = image[:, :, np.newaxis]
    image = np.repeat(image, 3, 2)
    Xmin = np.min(image)
    Xmax = np.max(image)
    image = 255 / (Xmax - Xmin) * (image - Xmin)
    cv2.imshow('depth', image)
    cv2.setMouseCallback("depth", on_EVENT_LBUTTONDOWN)
    cv2.waitKey(1)


if __name__ == '__main__':
    rospy.init_node('Get_depth', anonymous=True)
    rate = rospy.Rate(500)

    human_point_pub = rospy.Publisher('/Human_pose', Coord, queue_size=1)
    # rospy.Subscriber("/kinect2/sd/image_color_rect", Image, getPositionCallback)
    rospy.Subscriber("/kinect2/qhd/image_depth_rect", Image, show)
    rospy.spin()
