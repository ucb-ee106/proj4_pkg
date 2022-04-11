#!/usr/bin/env python
"""
Serial interface for the soft gripper
Author: Chris Correa
"""
import numpy as np
import serial
import os
import sys

import rospy
from proj4_pkg.msg import SoftGripperState, SoftGripperCmd

class SoftGripperSerialInterface():
    def __init__(self):
        """
        Serial interface for the soft gripper
        """

        # General setup
        self.state_pub = rospy.Publisher('soft_gripper_state', SoftGripperState, queue_size=10)
        self.subscriber = rospy.Subscriber('soft_gripper_cmd', SoftGripperCmd, self.send_cmd)
        rospy.on_shutdown(self.shutdown)
        rospy.sleep(1)

        self.setup_arduino()

    def setup_arduino(self):
        """
        Sets up communication with Arduino board
        """

        self.found_device = None
        # looks for re'/dev/ttyACM[0-9]'
        for device in os.listdir('/dev/'):
            if device.startswith('ttyACM'):
                self.found_device = device
        if self.found_device is None:
            rospy.logerr('Could not find Arduino board')
            rospy.signal_shutdown('Could not find Arduino board')
            sys.exit()
        self.ser = serial.Serial(os.path.join('/dev', self.found_device), 9600)
        
    def send_cmd(self, msg):
        """
        Callback function for gripper pump commands

        Parameters
        ----------
        msg : :obj:`proj4_pkg.SoftGripperCmd`
            gripper command message containing pump values
        """

        string = "\n"
        if msg.right >= 0:
            string = " r%d" % msg.right + string
        if msg.left >= 0:
            string = "l%d" % msg.left + string
        self.ser.write(string.encode())

    def run(self):
        """
        Listens for states across serial and publishes to ROS
        """

        rate = rospy.Rate(100)
        print_flag = True
        while not rospy.is_shutdown():
            try:
                read_serial = self.ser.readline()
                time = rospy.get_time()
                values = read_serial.decode('utf-8').split(',')
                left_pwm, left_pressure, left_flex, right_pwm, right_pressure, right_flex = [val.split('=')[1].split('p')[0].strip() for val in values]
                state = SoftGripperState(
                    time,
                    float(left_pwm), 
                    float(left_pressure), 
                    float(left_flex),
                    float(right_pwm),
                    float(right_pressure),
                    float(right_flex))
                if print_flag:
                    print('\nRunning soft gripper...')
                    print_flag = False
                self.state_pub.publish(state)
            except:
                print('Could not parse:', read_serial)
            rate.sleep()
        # Stop
        self.shutdown()

    def shutdown(self):
        """
        If you connected to an Arduino, stop it when exiting
        """

        if self.found_device:
            self.send_cmd(SoftGripperCmd())

if __name__ == '__main__':
    rospy.init_node('soft_gripper_serial_interface', anonymous=True)
    sgsi = SoftGripperSerialInterface()
    sgsi.run()
