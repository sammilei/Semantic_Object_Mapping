#!/usr/bin/env python

import sys, os
import rospy
from darknet_ros_msgs.msg import Object
from sensor_msgs.msg import Image, CameraInfo
from artifact_detection_rf.msg import RFDetection
from apriltags2_ros.msg import AprilTagDetectionArray, AprilTagDetection
from collections import deque
import Queue
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
import cv2
np.set_printoptions(threshold=sys.maxsize)
# Instantiate CvBridge
bridge = CvBridge()

class PublishRgbDepthDet:

  def __init__(self):
    self.robot_namespace = rospy.get_namespace()
    self.camera_names = rospy.get_param("camera_names")
    self.bridge = CvBridge()
    self.info_pubs = []
    self.rgb_pubs = []
    self.depth_pubs = []
    for name in self.camera_names:
      i = rospy.Publisher(self.robot_namespace + '/' + name + '/color/camera_info', CameraInfo, queue_size = 100)
      r = rospy.Publisher(self.robot_namespace + '/' + name + '/color/image_raw', Image, queue_size = 100)
      d = rospy.Publisher(self.robot_namespace + '/' + name + '/aligned_depth_to_color/image_raw', Image, queue_size = 100)
      self.info_pubs.append(i)
      self.rgb_pubs.append(r)
      self.depth_pubs.append(d)
    self.detection_pub = rospy.Publisher(self.robot_namespace + '/detected_object', Object, queue_size = 100)
    self.wifi_pub = rospy.Publisher(self.robot_namespace + '/wifi_rssi', RFDetection, queue_size = 100)
    self.at_pub = rospy.Publisher(self.robot_namespace + '/tag_detections', AprilTagDetectionArray, queue_size = 100)
    self.at_im_pub = rospy.Publisher(self.robot_namespace + '/tag_detections_image', Image, queue_size = 100)
    self.camera_index = 0
    self.rgb_image_cv = np.ones((240, 424, 3), np.uint8) * 100
    self.rgb_image_msg = self.bridge.cv2_to_imgmsg(self.rgb_image_cv, "bgr8")
    self.at_image_cv = np.ones((240, 424, 3), np.uint8) * 100
    self.at_image_msg = self.bridge.cv2_to_imgmsg(self.at_image_cv, "bgr8")
    self.depth_image_cv = np.ones((240, 424, 1), np.uint16) * 3000
    self.depth_image_msg = self.bridge.cv2_to_imgmsg(self.depth_image_cv, "mono16")
    self.publish_wifi = True
    self.secs = 0

  def create_camera_info_msg(self, camera):
    msg = CameraInfo()
    msg.height = 240
    msg.width = 424
    msg.distortion_model = 'plumb_bob'
    msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    msg.K = [306.45, 0.0, 212.73, 0.0, 306.45, 120.34, 0.0, 0.0, 1.0]
    msg.P = [306.45, 0.0, 212.73, 0.0, 0.0, 306.45, 120.34, 0.0, 0.0, 0.0, 1.0, 0.0]
    msg.header.frame_id = self.robot_namespace + camera + '/camera_color_optical_frame'
    return msg

  def create_rgb_image_msg(self, timestamp, camera):
    msg = self.rgb_image_msg
    msg.header.stamp = timestamp
    msg.header.frame_id = self.robot_namespace + camera + '/camera_color_optical_frame'
    return msg

  def create_depth_image_msg(self, timestamp, camera):
    msg = self.depth_image_msg
    msg.header.stamp = timestamp
    msg.header.frame_id = self.robot_namespace + camera + '/camera_color_optical_frame'
    return msg

  def create_yolo_detection_msg(self, timestamp, camera):
    msg = Object()
    msg.header.stamp = timestamp
    msg.camera_name = camera
    if camera == 'camera_front':
      msg.box.Class = "fire extinguisher"
    elif camera == 'camera_right':
      msg.box.Class = "backpack"
    else:
      msg.box.Class = 'survivor'      
    msg.box.xmin = 50
    msg.box.ymin = 50
    msg.box.xmax = 100
    msg.box.ymax = 100
    msg.box.probability = 0.8
    return msg

  def create_at_det_msg(self, timestamp, camera):
    msg = AprilTagDetectionArray()
    msg.header.stamp = timestamp
    msg.detections = []
    det1 = AprilTagDetection()
    det1.id = []
    det1.id.append(4)
    det1.size = []
    det1.size.append(0.2286)
    det1.pose.pose.pose.position.x = -0.5
    det1.pose.pose.pose.position.y = 0.0
    det1.pose.pose.pose.position.z = 1.0
    det1.pose.pose.pose.orientation.w = 1.0
    det1.pose.header.frame_id = self.robot_namespace.split('/')[1] + '/' + camera + '/camera_color_optical_frame'
    msg.detections.append(det1)
    det2 = AprilTagDetection()
    det2.id = []
    det2.id.append(6)
    det2.size = []
    det2.size.append(0.2286)
    det2.pose.pose.pose.position.x = 0.5
    det2.pose.pose.pose.position.y = 0.0
    det2.pose.pose.pose.position.z = 1.0
    det2.pose.pose.pose.orientation.w = 1.0
    det2.pose.header.frame_id = self.robot_namespace.split('/')[1] + '/' + camera + '/camera_color_optical_frame'
    msg.detections.append(det2)
    return msg

  def create_at_det_im_msg(self, timestamp, camera):
    msg = self.at_image_msg
    msg.header.stamp = timestamp
    msg.header.frame_id = self.robot_namespace + camera + '/camera_color_optical_frame'
    return msg

  def create_wifi_message(self, timestamp):
    msg = RFDetection()
    msg.header.stamp = timestamp
    msg.rssi = -45
    msg.id = 'PhoneArtifactMR2'
    return msg

  def publish_msgs(self):
    camera_name = self.camera_names[self.camera_index]
    timestamp = rospy.Time.now()
    i = self.create_camera_info_msg(camera_name)
    r = self.create_rgb_image_msg(timestamp, camera_name)
    d = self.create_depth_image_msg(timestamp, camera_name)
    y = self.create_yolo_detection_msg(timestamp, camera_name)
    w = self.create_wifi_message(timestamp)
    if camera_name == 'camera_front':
      a1 = self.create_at_det_msg(timestamp, camera_name)
      a2 = self.create_at_det_im_msg(timestamp, camera_name)
      self.at_pub.publish(a1)
      self.at_im_pub.publish(a2)
    self.info_pubs[self.camera_index].publish(i)
    # self.rgb_pubs[self.camera_index].publish(r)
    # if not camera_name == 'camera_front':
      # self.depth_pubs[self.camera_index].publish(d)
    # self.detection_pub.publish(y)
    if not (self.secs == timestamp.secs) and self.publish_wifi:
      self.wifi_pub.publish(w)

    self.camera_index = (self.camera_index +1)% len(self.camera_names)
    self.secs = timestamp.secs


def main(args):
  rospy.init_node('publish_rgb_depth_det', anonymous=True)
  p = PublishRgbDepthDet()
  rate = rospy.Rate(30)
  while not rospy.is_shutdown():
    p.publish_msgs()
    rate.sleep()

if __name__ == '__main__':
    main(sys.argv)