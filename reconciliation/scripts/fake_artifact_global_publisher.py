#!/usr/bin/env python

import yaml
import pprint
import os
import rospy
from artifact_msgs.msg import Artifact
from Queue import Queue
from std_msgs.msg import Header

pp = pprint.PrettyPrinter(indent=2)

# https://stackoverflow.com/questions/918154/relative-paths-in-python
dirname = os.path.dirname(__file__)
msg_yaml = os.path.join(dirname, '../cfg/dummy_artifact_global.yaml')

def publisher():
    pub = rospy.Publisher('/husky2/artifact_localization/unreconciled_artifact', Artifact, queue_size=1)
    rospy.init_node('fake_artifact_global_publisher', anonymous=True)
    rate = rospy.Rate(10)

    with open(msg_yaml, 'r') as f:
        data = yaml.safe_load(f)

    artifact_msg_queue = Queue()
    count = 0
    for m in data:
        a = Artifact()
        a.header = Header()
        a.id = m['id']
        a.point.header = Header()
        a.point.point.x = m['point']['point']['x']
        a.point.point.y = m['point']['point']['y']
        a.point.point.z = m['point']['point']['z']
        a.label = m['label']
        artifact_msg_queue.put(a)

    while not rospy.is_shutdown():
        try:
            rospy.sleep(1)
            msg = artifact_msg_queue.get()
            print 'Publishing message #', msg.id
            pub.publish(msg) 
        except rospy.ROSInterruptException:
            pass
       

if __name__ == '__main__':
    publisher()
