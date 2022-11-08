#!/usr/bin/env python

import time
import subprocess
import sys
import rospy
import os
from artifact_msgs.msg import WiFiScanRaw


class ChannelSelector:
    def __init__(self):
        self.robot_namespace = rospy.get_namespace().split('/')[1]
        self.interface_name = rospy.get_param('channel_scanner_interface_name')
        print 'Scanning on', self.interface_name
        self.hotspot_channels = {}
        self.pub_scan_raw = rospy.Publisher('wifi_scan_raw', WiFiScanRaw, queue_size=100)

    def update_channel_list(self):
        rospy.loginfo("[CS] Scanning to find any unexpected WiFi channels...")
        p = subprocess.Popen(['sudo', 'iwlist', self.interface_name, 'scan'],
                             stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        res, _ = p.communicate()
        lines = res.split('\n')
        data = {}
        address = ''
        channel = 0
        strength = 0
        id = ''
        for line in lines:
            if line.startswith('          Cell'):
                address = line.split('Address: ')[1]
            if line.startswith('                    Channel'):
                channel = int(line.split(':')[1])
            if line.startswith('                    Quality'):
                strength = int(line.split('Signal level=')[1].split(' dBm')[0])
            if line.startswith('                    ESSID'):
                id = line.split('ESSID:\"')[1].split('\"')[0]
            msg = WiFiScanRaw()
            msg.address = address
            msg.channel = channel
            msg.strength = strength
            msg.id = id
            msg.header.stamp = rospy.Time.now()
            self.pub_scan_raw.publish(msg)
            if self.is_cellphone(id) and not self.previously_seen(id):
                self.hotspot_channels.update({id: channel})
                rospy.loginfo(
                    "[CS] Found a new channel (%i) associated with %s", channel, id)
                rospy.set_param('additional_channels',
                                self.create_channel_set_from_dict())

    def is_cellphone(self, id):
        if id.startswith('PhoneArtifact'):
            return True
        else:
            return False

    def previously_seen(self, id):
        if id in self.hotspot_channels.keys():
            return True
        else:
            return False

    def create_channel_set_from_dict(self):
        # https://stackoverflow.com/questions/17218139/print-all-unique-values-in-a-python-dictionary
        return list(set(val for val in self.hotspot_channels.values()))


if __name__ == "__main__":
    cs = ChannelSelector()
    rospy.init_node('channel_selector', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        cs.update_channel_list()
        rate.sleep()
