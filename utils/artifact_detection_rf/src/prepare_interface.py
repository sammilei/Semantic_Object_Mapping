#!/usr/bin/env python

import time
import subprocess
import sys
import rospy
import os


def RunSystemCommand(command):
    print "Going to run", command
    result = os.system(command)
    if result == 0:
        print "Ran", command
        return True
    print "FAILED running", command
    raise Exception("ABORTING. FAILED running {}".format(command))


class PrepareInterface:
    def __init__(self):
        self.channel_scanner_interface_name = rospy.get_param('channel_scanner_interface_name')
        self.signal_scanner_interface_name = rospy.get_param('signal_scanner_interface_name')
        self.prepare_channel_scanner_interface(self.channel_scanner_interface_name)
        self.prepare_signal_scanner_interface(self.signal_scanner_interface_name)
        pass

    def prepare_channel_scanner_interface(self, interface):
        print "Setting up channel_scanner_interface_name={} to Managed mode".format(interface)
        RunSystemCommand('sudo rfkill unblock all')
        RunSystemCommand('sudo ifconfig ' + interface + ' down')
        RunSystemCommand('sudo iwconfig ' + interface + ' mode Managed')
        RunSystemCommand('sudo ifconfig ' + interface + ' up')
        pass

    def prepare_signal_scanner_interface(self, interface):
        print "Setting up signal_scanner_interface_name={} to monitor mode".format(interface)
        RunSystemCommand('sudo rfkill unblock all')
        RunSystemCommand('sudo ifconfig ' + interface + ' down')
        RunSystemCommand('sudo iwconfig ' + interface + ' mode monitor')
        RunSystemCommand('sudo ifconfig ' + interface + ' up')
        pass
    pass



if __name__ == "__main__":
    rospy.init_node('prepare_interface', anonymous=True)
    pi = PrepareInterface()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()
