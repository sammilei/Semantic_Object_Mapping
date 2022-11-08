#!/usr/bin/env python

import sys, os
import rospy
from sensor_msgs.msg import Image, PointCloud2
from darknet_ros_msgs.msg import Object
import numpy as np 
import cv2
from cv_bridge import CvBridge
import math

class topic_repeater:

  def __init__(self):
    self.robot_namespace = rospy.get_namespace()
    self.buffer_size = 5.0 * 1e9 # nsecs
    self.width = 424
    self.height = 240
    self.dummy_image = np.zeros((self.height, self.width, 1), dtype = "uint16")
    self.bridge = CvBridge()
    self.camera_names = ["camera_left", "camera_front", "camera_right"]
    
    # Subscribers
    self.image_subs = []
    self.image_buffers = {}
    self.repeater_pubs = {}
    for name in self.camera_names:
      self.image_subs.append(rospy.Subscriber(self.robot_namespace + name + "/aligned_depth_to_color/image_raw", Image, self.image_clbk))
      self.image_buffers.update({name: []})
      self.repeater_pubs.update({name: rospy.Publisher(self.robot_namespace + name + "/aligned_depth_to_color/image_raw_repeater", Image, queue_size = 100)})
    self.detection_sub = rospy.Subscriber(self.robot_namespace + "detected_object", Object, self.detection_clbk)

  def delete_old_messages(self, queue, cutoff):
    pruned = []
    for msg in queue:
      if self.difference_two_times(self.convert_rospy_stamp_to_nsec(msg.header.stamp), cutoff) >= 0.0:
        pruned.append(msg)
    return pruned

  def convert_rospy_stamp_to_nsec(self, time_ros):
    return time_ros.to_nsec()

  def difference_two_times(self, time1, time2):
    # times [int] [nsec]
    return time1 - time2

  def can_find_image_with_same_timestamp_as_detection(self, stamp, image_buffer):
    for msg in image_buffer:
      # print 'in can find same', msg.header.stamp, stamp
      if self.convert_rospy_stamp_to_nsec(msg.header.stamp) == stamp:
        return True
    return False

  def detection_clbk(self, msg):
    detection_stamp = self.convert_rospy_stamp_to_nsec(msg.header.stamp)
    camera_name = msg.camera_name
    if not self.can_find_image_with_same_timestamp_as_detection(detection_stamp, self.image_buffers[msg.camera_name]):
      rospy.loginfo("Publishing an artificial depth message for %s at %i", camera_name, detection_stamp) 
      msg = self.create_repeater_msg(detection_stamp, camera_name)
      self.repeater_pubs[camera_name].publish(msg)

  def prune_queues(self):
    cutoff = self.convert_rospy_stamp_to_nsec(rospy.Time.now()) - self.buffer_size
    for name in self.camera_names:
      # print name, len(self.image_buffers[name])
      self.image_buffers[name] = self.delete_old_messages(self.image_buffers[name], cutoff)
    
  def get_camera_name_from_frame(self, frame):
    # "husky1/camera_front/camera_color_optical_frame" --> "camera_front"
    if frame[0] == "/": # shouldn't happen
      frame = frame[1:]
    return frame.split("/")[1]

  def image_clbk(self, msg):
    camera_name = self.get_camera_name_from_frame(msg.header.frame_id)
    self.image_buffers[camera_name].append(msg)

  def convert_nsec_to_rospy_stamp(self, time_nsec):
    nsec = 0
    sec = 0
    for x in range(9): # get all the nsecs
      rem = (time_nsec % 10) * math.pow(10, x)
      nsec += rem
      time_nsec = time_nsec //10
      sec = time_nsec
    rospy_time = rospy.Time()
    rospy_time.secs = sec
    rospy_time.nsecs = nsec
    return rospy_time


  def create_repeater_msg(self, nsec, camera_name):
    # header: 
    #   seq: 32556
    #   stamp: 
    #     secs: 1581440590
    #     nsecs: 529239045
    #   frame_id: "husky1/camera_front/camera_color_optical_frame"
    # height: 240
    # width: 424
    # encoding: "16UC1"
    # is_bigendian: 0
    # step: 848
    # data: "<array type: uint8, length: 203520>"
    msg = self.bridge.cv2_to_imgmsg(self.dummy_image, "mono16")
    msg.header.stamp = self.convert_nsec_to_rospy_stamp(nsec)
    msg.header.frame_id = self.robot_namespace.split("/")[1] + "/" + camera_name + "/camera_color_optical_frame"
    msg.height = self.height
    msg.width = self.width
    msg.encoding = "16UC1"
    msg.is_bigendian = 0
    msg.step = 848
    return msg

def main(args):
  rospy.init_node('topic_repeater', anonymous=True)
  tr = topic_repeater()
  r = rospy.Rate(10)
  while not rospy.is_shutdown():
    tr.prune_queues()
    r.sleep()

if __name__ == '__main__':
    main(sys.argv)