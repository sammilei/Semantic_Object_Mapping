#!/usr/bin/env python

import sys
import os
import rospy
from wifi_bt_detector.msg import RFDetection
from matplotlib import pyplot as plt
import numpy as np 
from visualization_msgs.msg import Marker


class parse_bag:

  def __init__(self):
    self.sub = rospy.Subscriber("/husky/wifi_rssi",RFDetection,self.callback)
    self.times = []
    self.rssi = []
    self.fig, self.ax = plt.subplots()
    self.pub = rospy.Publisher('wifi_rssi_marker', Marker, queue_size = 100)
    self.id = 0

  def callback(self, msg):
    if msg.id == 'PhoneArtifact99':
      self.times.append(msg.header.stamp.secs + msg.header.stamp.nsecs / 1e9)
      self.rssi.append(msg.rssi)
      print msg.header.stamp.secs + msg.header.stamp.nsecs / 1e9, msg.rssi

      marker = Marker()
      marker.header.frame_id = "/husky/base_link"

      marker.type = marker.POINTS
      marker.action = marker.ADD
      marker.pose.orientation.w = 1

      # scale
      max_rssi = -30
      min_rssi = -80
      scale = float(msg.rssi - min_rssi) / float(max_rssi - min_rssi)
      marker.type = Marker.SPHERE
      marker.id = self.id
      marker.lifetime.secs = 15
      marker.scale.x = scale
      marker.scale.y = scale
      marker.scale.z = scale
      marker.color.a = 1.0
      marker.color.r = 1.0

      self.pub.publish(marker)

      self.id += 1
    # self.ax.scatter(self.times, self.rssi)

  def show_plot(self):
    plt.show()

def main(args):
  pb = parse_bag()
  rospy.init_node('parse_bag', anonymous=True)
  try:
    rospy.spin()
    # pb.show_plot()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)