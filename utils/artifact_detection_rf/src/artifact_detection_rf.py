#!/usr/bin/python3

import time
import subprocess
import sys
import rospy
import os
from artifact_msgs.msg import PointSourceDetection
from std_msgs.msg import Header
import pyshark


class RSSIPublisher:
    def __init__(self):
        self.robot_namespace = rospy.get_namespace().split('/')[1]
        self.pub_wifi = rospy.Publisher('/' + self.robot_namespace + '/wifi_rssi', PointSourceDetection, queue_size=1)
        self.capture = pyshark.LiveCapture(interface='wpanda0')

    def get_rssi(self):
        for packet in self.capture.sniff_continuously(packet_count=5):
            layers = packet.get_multiple_layers('wlan')
            if len(layers) > 1:
                wlan_data = layers[1]
                if hasattr(wlan_data, 'ssid'):
                    if not wlan_data.ssid.find("PhoneArtifact") == -1:
                        msg = PointSourceDetection()
                        msg.header = Header()
                        msg.header.stamp = rospy.Time.now()
                        msg.header.frame_id = self.robot_namespace + '/base_link'
                        msg.id = wlan_data.ssid
                        msg.strength = int(packet.radiotap.dbm_antsignal)
                        self.pub_wifi.publish(msg)
                        print (msg.header.stamp, wlan_data.ssid, packet.radiotap.dbm_antsignal)

if __name__ == "__main__":
    print("Initializing Cellphone WiFi detector...")
    rp = RSSIPublisher()
    rospy.init_node('cellphone_wifi_detector', anonymous=True)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        try:
            rp.get_rssi()
            rate.sleep()
        except KeyboardInterrupt:
            break
