#!/usr/bin/env python

import sys, os
import rospy
import serial
from geometry_msgs.msg import PointStamped
import csv



class total_station_points:

  def __init__(self):
    self.ser = serial.Serial('/dev/ttyUSB0', 9600)
    self.pub = rospy.Publisher('total_station', PointStamped, queue_size = 1)
    self.points = {}

  def listen_to_serial(self):
    while True:
        line = self.ser.readline()
        id = line.split(',')[0]
        p = PointStamped()
        p.header.stamp = rospy.Time.now()
        p.x = float(line.split(',')[1])
        p.y = float(line.split(',')[2])
        p.z = float(line.split(',')[3])
        if not id in self.points.keys():
            print 'Adding and publishing', id, '\n', p
            self.points.update({id: p})
            self.pub.publish(p)      

def main(args):
  rospy.init_node('total_station_points', anonymous=True)
  ts = total_station_points()
  try:
    ts.listen_to_serial()
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)    