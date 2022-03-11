#!/home/lilinux/anaconda3/envs/yolov5/bin/python

from std_msgs.msg import String
import json
import math
from wpb_home_behaviors.msg import Coord
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rospy
import numpy as np
import cv2
import random

LENGTH = 0.6
TAN = 1 / LENGTH
ALPHA = 3

g_allO = []
g_N = None

_cv_bridge = CvBridge()

sub_topic = "/kinect2/qhd/image_depth_rect"

class_list = ['bottle']

if 'sd' in sub_topic:
    CAMERA_WIDTH = 512
    CAMERA_HEIGHT = 424
elif 'qhd' in sub_topic:
    CAMERA_WIDTH = 960
    CAMERA_HEIGHT = 540
elif 'hd' in sub_topic:
    CAMERA_WIDTH = 1920
    CAMERA_HEIGHT = 1080
else:
    raise IndexError


def pixel2real(pixel_x, pixel_y, z):
    norm_pixel_x = pixel_x - (CAMERA_WIDTH / 2)
    return norm_pixel_x / (CAMERA_HEIGHT / 2) * z / TAN


def pixel2relative(pixel_x, pixel_y, z):
    relative_y = pixel_y / CAMERA_HEIGHT
    return relative_y


class Entities:
    def __init__(self, class_list=['bottle']):
        self.depth = None
        self.objects = []
        self.class_list = class_list

    def get_depth(self, pixel_x, pixel_y):
        index_x = int(pixel_x)
        index_y = int(pixel_y)
        if isinstance(self.depth, np.ndarray):
            return self.depth[index_y - ALPHA:index_y + ALPHA, index_x - ALPHA:index_x + ALPHA].mean()
        else:
            return -1

    def add(self, temp):
        self.objects = []
        for cla, ets in temp.items():
            if cla in self.class_list:
                for et in ets:
                    pixel_x, pixel_y, pixel_w, pixel_h, _ = et
                    norm_pixel_x = pixel_x - (CAMERA_WIDTH / 2)
                    z = self.get_depth(pixel_x, pixel_y) / 1000
                    d_angle = - math.atan(norm_pixel_x / (CAMERA_HEIGHT / 2) /
                                          TAN) if abs(norm_pixel_x) > 10 else 0
                    self.objects.append([cla, pixel_x, pixel_y, z, d_angle])

    def getRandomEntity(self):
        return random.choice(self.objects)

    def getAllEntity(self):
        return self.objects

    def getNearEntity(self):
        pointer = []
        minn_d_angle = 5
        for object in self.objects:
            if abs(object[4]) < minn_d_angle:
                minn_d_angle = abs(object[4])
                pointer = object
        return pointer


global_entities = Entities(class_list)


def update_depth(image_msg):
    def draw(object, image, text=True, rect=True, color=(0, 0, 255)):
        pixel_x = object[1]
        pixel_y = object[2]
        if rect:
            cv2.rectangle(image, (int(pixel_x), int(pixel_y)),
                          (int(pixel_x + 5), int(pixel_y + 5)), color, 2)
        if text:
            image = cv2.putText(image, str(object[0]) + "%.2f" % object[3], (int(pixel_x), int(pixel_y)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        return image

    image = _cv_bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
    _, image = cv2.threshold(
        image, 4000, 4000, cv2.THRESH_TRUNC)
    global_entities.depth = image
    # image = cv2.resize(image, (106, 128))
    image = image[:, :, np.newaxis]
    image = np.repeat(image, 3, 2)
    Xmin = np.min(image)
    Xmax = np.max(image)
    image = 255 / (Xmax - Xmin) * (image - Xmin)
    global g_allO, g_N
    for object in g_allO:
        image = draw(object, image)
    if g_N:
        image = draw(g_N, image, text=False, color=(255, 255, 0))
        cv2.imshow('depth', image)
    cv2.waitKey(10)


def packCoord(object):
    cla, x, y, z, d_angle = object
    pose_msg = Coord()
    pose_msg.name = cla
    pose_msg.x = [pixel2real(x, y, z)]
    pose_msg.y = [pixel2relative(x, y, z)]
    pose_msg.z = [z]
    pose_msg.probability = [d_angle]
    return pose_msg


def yoloCallback(msg: String):
    global g_allO, g_N
    s = msg.data
    result = json.loads(s)
    global_entities.add(result)
    g_allO = global_entities.getAllEntity()
    for object in g_allO:
        pose_msg = packCoord(object)
        yolo_pub.publish(pose_msg)
    g_N = global_entities.getNearEntity()
    if g_N:
        pose_msg = packCoord(g_N)
        yolo_pub.publish(pose_msg)
        rospy.set_param('rubbish_exist', 1)
    rate.sleep()


if __name__ == '__main__':
    rospy.init_node('Yolo_process', anonymous=True)
    rate = rospy.Rate(500)
    yolo_pub = rospy.Publisher('/rubbishes', Coord, queue_size=1)

    rospy.Subscriber(name="yolo_result", data_class=String,
                     queue_size=5, callback=yoloCallback)

    rospy.Subscriber(sub_topic, Image, update_depth)

    rospy.spin()
