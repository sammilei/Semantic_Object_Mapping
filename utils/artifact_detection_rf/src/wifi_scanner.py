#!/usr/bin/env python

import time
import subprocess
import sys
import rospy
import os
from core_msgs.msg import WiFiScanRaw
from core_msgs.msg import PointSourceDetection
from std_msgs.msg import Int8MultiArray


class WiFiScanner:
    def __init__(self):
        self.robot_namespace = rospy.get_namespace().split('/')[1]
        self.interface_name = rospy.get_param('channel_scanner_interface_name')
        print 'wifi_scanner.py: Scanning on', self.interface_name
        self.pub_scan_raw = rospy.Publisher('wifi_scan_raw', WiFiScanRaw, queue_size=1)
        self.pub_artifact_rssi = rospy.Publisher('wifi_rssi', PointSourceDetection, queue_size=1)

    def scan(self):
        scan_time = rospy.Time.now()
        rospy.loginfo("[WS] Scanning to find WiFi channels currently in range...")
        p = subprocess.Popen(['sudo', 'iwlist', self.interface_name, 'scan'],
                             stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        res, _ = p.communicate()
        if p.returncode != 0:
            print "Failed to get iwlist for {}".format(self.interface_name)
            return
        lines = res.split('\n')
        cell_block_start_line_numbers = []
        line_number = 0
        for line in lines:
            if line.startswith('          Cell'):
                cell_block_start_line_numbers.append(line_number)
            line_number += 1

        active_channels = []

        # Extract each cell block
        cell_blocks = []
        for i in range(len(cell_block_start_line_numbers)):
            if i == len(cell_block_start_line_numbers) - 1:
                cell_block = lines[cell_block_start_line_numbers[i]:len(lines)]
            else:
                cell_block = lines[cell_block_start_line_numbers[i]:cell_block_start_line_numbers[i+1]]
            cell_blocks.append(cell_block)

        # Process each block
        for block in cell_blocks:
            for line in block: 
                if line.startswith('          Cell'):
                    address = line.split('Address: ')[1]
                if line.startswith('                    Channel'):
                    channel = int(line.split(':')[1])
                if line.startswith('                    Quality'):
                    strength = int(line.split('Signal level=')[1].split(' dBm')[0])
                if line.startswith('                    ESSID'):
                    id_ = line.split('ESSID:\"')[1].split('\"')[0]  # id is a builtin function

            # Create WiFiScanRaw message with all detected hotspots
            msg = WiFiScanRaw()
            msg.address = address
            msg.channel = channel
            msg.strength = strength
            msg.id = id_
            msg.header.stamp = rospy.Time.now()
            self.pub_scan_raw.publish(msg)

            # Get the channels of cell phone artifacts in range
            if self.is_cellphone(id_):
                active_channels.append(channel)
                rospy.loginfo(
                    "[WS] Listening to channel (%i) associated with %s", channel, id_)
                msg = PointSourceDetection()
                msg.header.stamp = scan_time
                msg.header.frame_id = self.robot_namespace + '/base_link'
                msg.id = id_
                msg.strength = strength
                self.pub_artifact_rssi.publish(msg)
            
        # Publish active channels
        rospy.set_param('active_channels', active_channels)


    def is_cellphone(self, id_):
        if id_.startswith('PhoneArtifact'):
            return True
        else:
            return False


if __name__ == "__main__":
    ws = WiFiScanner()
    rospy.init_node('wifi_scanner', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        ws.scan()
        rate.sleep()
