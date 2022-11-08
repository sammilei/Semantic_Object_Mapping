#!/usr/bin/env python

import sys, os
import rospy
from sensor_msgs.msg import Image, PointCloud2

class topic_switcher:

  def __init__(self):
    self.robot_namespace = rospy.get_namespace()
    
    # Subscribers
    self.depth_front_sub = rospy.Subscriber(self.robot_namespace + "camera_front/aligned_depth_to_color/image_raw", Image, self.depth_front_callback)
    self.depth_left_sub = rospy.Subscriber(self.robot_namespace + "camera_left/aligned_depth_to_color/image_raw", Image, self.depth_left_callback)
    self.depth_right_sub = rospy.Subscriber(self.robot_namespace + "camera_right/aligned_depth_to_color/image_raw", Image, self.depth_right_callback)

    # Publishers
    self.depth_front_pub_redir = rospy.Publisher(self.robot_namespace + "camera_front/aligned_depth_to_color/image_raw_redir", Image, queue_size = 100)
    self.depth_front_pub_drain = rospy.Publisher(self.robot_namespace + "camera_front/aligned_depth_to_color/image_raw_drain", Image, queue_size = 100)
    self.depth_left_pub_redir = rospy.Publisher(self.robot_namespace + "camera_left/aligned_depth_to_color/image_raw_redir", Image, queue_size = 100)
    self.depth_left_pub_drain = rospy.Publisher(self.robot_namespace + "camera_left/aligned_depth_to_color/image_raw_drain", Image, queue_size = 100)
    self.depth_right_pub_redir = rospy.Publisher(self.robot_namespace + "camera_right/aligned_depth_to_color/image_raw_redir", Image, queue_size = 100)
    self.depth_right_pub_drain = rospy.Publisher(self.robot_namespace + "camera_right/aligned_depth_to_color/image_raw_drain", Image, queue_size = 100)

  def depth_front_callback(self, msg):
    if rospy.get_param("topic_switch/camera_front_depth") == True:
      self.depth_front_pub_redir.publish(msg)
    else: 
      self.depth_front_pub_drain.publish(msg)

  def depth_left_callback(self, msg):
    if rospy.get_param("topic_switch/camera_left_depth") == True:
      self.depth_left_pub_redir.publish(msg)
    else: 
      self.depth_left_pub_drain.publish(msg)

  def depth_right_callback(self, msg):
    if rospy.get_param("topic_switch/camera_right_depth") == True:
      self.depth_right_pub_redir.publish(msg)
    else: 
      self.depth_right_pub_drain.publish(msg)

def main(args):
  rospy.init_node('topic_switcher', anonymous=True)
  ts = topic_switcher()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
