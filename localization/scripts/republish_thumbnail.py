#!/usr/bin/env python

import sys, os
import rospy
from artifact_msgs.msg import Artifact
from sensor_msgs.msg import Image

class republish_thumbnail:

  def __init__(self):
    self.robot_namespace = rospy.get_namespace()
    self.sub = rospy.Subscriber(self.robot_namespace + "artifact",Artifact,self.callback)
    self.pub = rospy.Publisher(self.robot_namespace + 'artifact_thumbnail', Image, queue_size = 100)
    self.id = 0

  def callback(self, msg):
    im = msg.thumbnail
    self.pub.publish(im)

def main(args):
  rt = republish_thumbnail()
  rospy.init_node('republish_thumbnail', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
