#!/usr/bin/env python

import sys, os
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import time
from copy import deepcopy

# from message_filters import ApproximateTimeSynchronizer, Subscriber



class republish_thumbnail:

  def __init__(self):
    self.robot_namespace = "husky2/"#rospy.get_namespace()
    self.div_h = 2
    self.div_v = 3
    self.sub = rospy.Subscriber(self.robot_namespace + "camera_right/color/image_raw",Image,self.callback)
    self.pub = rospy.Publisher(self.robot_namespace + 'camera_right/color/image_raw_subdiv', Image, queue_size = 100)
    self.bridge = CvBridge()
    self.seq = 0
    self.first_received = False

  def callback(self, msg):
    if not self.first_received:
        self.seq = msg.header.seq
        self.first_received = True
    header = msg.header
    if header.stamp.nsecs > 1e9 - self.div_h * self.div_v - 1:
      header.stamp.nsecs = max(1e9 - self.div_h * self.div_v - 1, header.stamp.nsecs) # make it so that when nanosec increments are added later that it doesn't overflow to next sec
    cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    for i in range(self.div_h):
      for j in range(self.div_v):
        incr = self.get_increment(i, j)
        subdiv = self.subdivide_image(cv_image, i, j)
        msg_subdiv = self.bridge.cv2_to_imgmsg(subdiv, "bgr8")
        msg_subdiv.header = deepcopy(header)
        msg_subdiv.header.stamp.nsecs = header.stamp.nsecs + incr * 1e3
        msg_subdiv.header.seq = self.seq + incr
        time.sleep(0.025)
        self.pub.publish(msg_subdiv)

  def subdivide_image(self, cv_image, i, j):
    (rows,cols,channels) = cv_image.shape
    subdiv = cv_image[rows / self.div_h * i : rows / self.div_h * (i+1), cols / self.div_v * j : cols / self.div_v * (j+1)]
    return subdiv

  def get_increment(self, i, j):
    return i * self.div_v + j

def main(args):
  rt = republish_thumbnail()
  rospy.init_node('republish_thumbnail', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)