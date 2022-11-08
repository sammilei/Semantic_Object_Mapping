#!/usr/bin/env python
# simple code to spawn packet capture and publish
#
import pyshark
from std_msgs.msg import Header
import os
# import time
import subprocess
import rospy
import atexit
import sys
from artifact_msgs.msg import PointSourceDetection
import datetime
import signal


class RSSIPublisher:
    def __init__(self):
        self.robot_namespace = rospy.get_namespace().split('/')[1]
        self.interface_name = rospy.get_param('signal_scanner_interface_name')
        self.capture_window = str(rospy.get_param('capture_window'))
        self.channels = rospy.get_param('channels')
        self.channel_index = 0
        self.channel = self.channels[self.channel_index]
        self.pub_wifi = rospy.Publisher('wifi_rssi', PointSourceDetection, queue_size=1)
        # tshark -i ${INTERFACE_NAME} -T fields -e frame.time -e wlan.ssid -e radiotap.dbm_antsignal -e wlan_radio.channel
        self.command = ['tshark', '-i', self.interface_name, '-T', 'fields', '-e', 'frame.time', '-e', 'wlan.ssid',
                   '-e', 'radiotap.dbm_antsignal', '-e', 'wlan_radio.channel', '-a', 'duration:' + self.capture_window]

    def check_for_active_channels(self):
        if rospy.has_param('active_channels'):
            active_channels = rospy.get_param('active_channels')
            for channel in active_channels:
                if channel not in self.channels:
                    self.channels.append(channel)
                    rospy.loginfo("[RP] Adding unexpected channel %i", channel)
                    pass
                pass
            pass
        pass

    def set_channel(self, channel):
        result = os.system('sudo iw dev ' + self.interface_name + ' set channel ' + str(channel))
        if result == 0:
            rospy.loginfo("[RP] Listening to channel %i", channel)
            pass
        else:
            rospy.loginfo("[RP] FAILED TO change to channel %i", channel)
            pass
        pass

    def change_channel(self):
        self.channel_index = (self.channel_index + 1) % len(self.channels)
        self.set_channel(self.channels[self.channel_index])
        pass

    def get_rssi(self):
        p = subprocess.Popen(self.command, stdout=subprocess.PIPE)
        out, err = p.communicate()
        if p.returncode != 0:
            rospy.loginfo("[RP] get_rssi command FAILED")
            return
        for line in out.splitlines():
            segments = line.split('\t')
            time = rospy.Time.from_sec(self.utc_to_unix(segments[0]))
            id_ = segments[1]
            rssi = segments[2]
            channel = segments[3]
            if (self.is_cellphone(id_)):
                self.publish(time, id_, rssi)
                pass
            pass
        self.change_channel()
        pass

    def is_cellphone(self, id_):
        if id_.startswith('PhoneArtifact'):
            return True
        else:
            return False
        pass

    def publish(self, time, id_, rssi):
        msg = PointSourceDetection()
        msg.header = Header()
        msg.header.stamp = time
        msg.header.frame_id = self.robot_namespace + '/base_link'
        msg.id = id_
        msg.strength = int(rssi)
        self.pub_wifi.publish(msg)
        rospy.loginfo("[RP] %s.%s %s %i", msg.header.stamp.secs, msg.header.stamp.nsecs, msg.id, msg.strength)

    # TODO: simplify all this utc_to_unix stuff. Use the functions in time and datetime.
    def utc_to_unix(self, utc):
        month = self.get_month_index_from_str(utc[0:3])
        day = utc[4:6]
        year = utc[8:12]
        hour = utc[13:15]
        minute = utc[16:18]
        sec = utc[19:21]
        nsec = utc[22:28]
        # https://stackoverflow.com/questions/19801727/convert-datetime-to-unix-timestamp-and-convert-it-back-in-python
        unix = datetime.datetime.strptime(year + month + day + 'T' + hour + minute + sec + '.' + nsec, '%Y%m%dT%H%M%S.%f')
        return (unix - datetime.datetime(1970, 1, 1)).total_seconds() + 60 * 60 * 8  # magic numbers account for 8 hr PST offset in secs

    def get_month_index_from_str(self, month_str):
        if month_str == 'Jan':
            return '01'
        elif month_str == 'Feb':
            return '02'
        elif month_str == 'Mar':
            return '03'
        elif month_str == 'Apr':
            return '04'
        elif month_str == 'May':
            return '05'
        elif month_str == 'Jun':
            return '06'
        elif month_str == 'Jul':
            return '07'
        elif month_str == 'Aug':
            return '08'
        elif month_str == 'Sep':
            return '09'
        elif month_str == 'Oct':
            return '10'
        elif month_str == 'Nov':
            return '11'
        elif month_str == 'Dec':
            return '12'
        pass


if __name__ == "__main__":
    print("Initializing Cellphone WiFi detector...")
    rp = RSSIPublisher()
    rospy.init_node('cellphone_wifi_detector', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            rp.get_rssi()
            rp.check_for_active_channels()
            rate.sleep()
        except KeyboardInterrupt:
            break
