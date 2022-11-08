#!/usr/bin/env python

import time
import subprocess
import sys
import rospy
import os
from core_msgs.msg import PointSourceDetection
from core_msgs.msg import WiFiScanRaw
from std_msgs.msg import Header, Empty


class BTScanner:
    def __init__(self):
        self.last_bt_stamp = rospy.Time()
        self.get_index()
        self.robot_namespace = rospy.get_namespace().split('/')[1]
        self.pub_bt = rospy.Publisher(rospy.get_namespace() + 'bt_rssi', PointSourceDetection, queue_size=100)
        self.pub_bt_raw = rospy.Publisher(rospy.get_namespace() + 'bt_scan_raw', WiFiScanRaw, queue_size=100)
        self.pub_heartbeat = rospy.Publisher(rospy.get_namespace() + 'bt_heartbeat', Empty, queue_size=1)

    def get_index(self):
        p = subprocess.Popen(['sudo', 'hcitool', 'dev'], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        res, _ = p.communicate()
        lines = res.decode().split('\n')
        print(lines)
        for line in lines:
            if line.endswith('00:1A:7D:DA:71:15'):  # and 'hci' in line:
                self.index = str(line[4])
                print('bt device with index = ', self.index)
                break

    def get_rssi(self):
        scan_time = rospy.Time.now()
        p = subprocess.Popen(['sudo', 'btmgmt', '--index', self.index, 'find'], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        res, _ = p.communicate()
        lines = res.decode().split('\n')
        data = {}
        readings = []
        cell_block_start_line_numbers = []
        cell_blocks = []
        line_number = 0
        for line in lines:
            if line.startswith('hci' + self.index + ' dev_found: '):
                cell_block_start_line_numbers.append(line_number)
            line_number += 1

        for i in range(len(cell_block_start_line_numbers)):
            if i == len(cell_block_start_line_numbers) - 1:
                cell_block = lines[cell_block_start_line_numbers[i]:len(lines)]
            else:
                cell_block = lines[cell_block_start_line_numbers[i]:cell_block_start_line_numbers[i + 1]]
            cell_blocks.append(cell_block)

        for block in cell_blocks:
            name_line_found = False
            for line in block:
                if line.startswith('hci' + self.index + ' dev_found: '):
                    rssi_str = line.split('rssi ')[1].split(' flags')[0]
                    rssi = int(rssi_str)
                    address = line.split('found: ')[1].split(' type')[0]
                if line.startswith('name '):
                    id_ = line.split(' ')[1]
                    name_line_found = True
            if name_line_found:
                if id_.startswith('PhoneArtifact') or id_.startswith('CubeArtifact'):
                    rospy.loginfo("[BT Det] Preparing message for %s (%idBm)", id_, rssi)
                    msg = PointSourceDetection()
                    msg.header.stamp = scan_time
                    msg.id = id_
                    msg.strength = rssi
                    self.pub_bt.publish(msg)
                msg = WiFiScanRaw()
                msg.header.stamp = scan_time
                msg.header.frame_id = self.robot_namespace + '/base_link'
                msg.id = id_
                msg.strength = rssi
                msg.address = address
                self.pub_bt_raw.publish(msg)

                self.last_bt_stamp = scan_time

    def publish_heartbeat(self, event):
        meas_timeout = rospy.Duration(30.0)
        bt_delay = rospy.Time.now() - self.last_bt_stamp
        if bt_delay < meas_timeout:
            rospy.loginfo_once("Received bluetooth measurement. Publishing health heartbeat")
            self.pub_heartbeat.publish(Empty())
        else:
            if self.last_bt_stamp:
                rospy.logwarn_throttle(
                    10.0,
                    "No bluetooth measurement for the last %.1f secs",
                    bt_delay.to_sec(),
                )
            else:
                rospy.logwarn_throttle(
                    10.0,
                    "Haven't received any bluetooth measurement",
                )


if __name__ == "__main__":
    print("Initializing Bluetooth scanner...")
    bts = BTScanner()
    rospy.init_node('bt_scanner', anonymous=True)
    rate = rospy.Rate(1)
    hearbeat_timer = rospy.Timer(rospy.Duration(1.0), bts.publish_heartbeat)
    while not rospy.is_shutdown():
        bts.get_rssi()
        rate.sleep()
