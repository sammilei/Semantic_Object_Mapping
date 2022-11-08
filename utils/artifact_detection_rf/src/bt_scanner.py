#!/usr/bin/env python
import serial
import rospy
from core_msgs.msg import PointSourceDetection
from core_msgs.msg import WiFiScanRaw
from std_msgs.msg import Header, Empty

class BTScanner:
    def __init__(self):
        self.pipe_serial = None
        self.is_pipe_alive = False
        self.last_bt_stamp = rospy.Time()
        self.robot_namespace = rospy.get_namespace().split('/')[1]

        self.pub_bt = rospy.Publisher(rospy.get_namespace() + 'bt_rssi', PointSourceDetection, queue_size=100)
        self.pub_bt_raw = rospy.Publisher(rospy.get_namespace() + 'bt_scan_raw', WiFiScanRaw, queue_size=100)
        self.pub_heartbeat = rospy.Publisher(rospy.get_namespace() + 'bt_heartbeat', Empty, queue_size=1)
        
        self.open_serial()

    def open_serial(self):
        try:
            self.pipe_serial = serial.Serial(port='/dev/ttyESP32', baudrate=115200, timeout=.1)
            self.is_pipe_alive = True
        except:
            rospy.logfatal("[BT Det] Failed to opening serial")

    def get_bt_info(self):
        try:
            if(self.is_pipe_alive is False):
                rospy.logfatal("[BT Det] Serial port is closed")
                self.open_serial()
                return
            raw_serial = str(self.pipe_serial.readline()).encode("utf-8").split(",")
            scan_time = rospy.Time.now()
            if(len(raw_serial) < 2):
                self.last_bt_stamp = scan_time
                return

            id_ = raw_serial[0]
            rssi = int(raw_serial[1])
            address = raw_serial[2][:-1]

            if(rssi < -70):
                rospy.loginfo("[BT Det] Signal rejected %s (%idBm)", id_, rssi)
                return
            
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
        except:
            self.is_pipe_alive = False
            rospy.logfatal("[BT Det] Error occurred in get_bt_info")

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
    bts.open_serial()
    rospy.init_node('bt_scanner', anonymous=True)
    rate = rospy.Rate(10)
    hearbeat_timer = rospy.Timer(rospy.Duration(1.0), bts.publish_heartbeat)
    
    while not rospy.is_shutdown():
        bts.get_bt_info()
        rate.sleep()
