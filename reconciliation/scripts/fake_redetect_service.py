#!/usr/bin/env python

from darknet_ros_msgs.srv import Detect, DetectResponse
from darknet_ros_msgs.msg import Object
import rospy


def handle_detection(req):
    detection = Object()
    detection.box.probability = 1.0
    detection.box.yolo_probability = 1.0
    return DetectResponse([detection])


def dummy_redetect_server():
    rospy.init_node('fake_detection_service')
    s1 = rospy.Service('detection_service_rgb', Detect, handle_detection)
    s2 = rospy.Service('detection_service_therm', Detect, handle_detection)
    rospy.spin()


if __name__ == "__main__":
    dummy_redetect_server()
