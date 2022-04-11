#!/usr/bin/env python
"""
Grasp Controller
Author: Josephine Koe
"""
import numpy as np
import pandas as pd
import sys
import os
import traceback

import rospy
import argparse
import time
import tf2_ros
import rosbag

from soft_gripper_user_interface import SoftGripperUserInterface
from proj4_pkg.msg import SoftGripperCmd, SoftGripperState
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression
from matplotlib.animation import FuncAnimation

class GraspController():
    def __init__(self):
        """
        Grasp Controller
        """

        self.send_gripper_cmd = SoftGripperUserInterface().send_gripper_cmd
        rospy.sleep(1)
        rospy.Subscriber("soft_gripper_state", SoftGripperState, self.update_state)
        rospy.on_shutdown(self.shutdown)
        self.rate = rospy.Rate(100)
        self.desired_angle = 0
        self.current_state = SoftGripperState(0, 0, 0, 0, 0, 0, 0)

    def update_state(self, msg):
        """
        Updates the current state from the soft_gripper_state topic

        Parameters
        ----------
        msg : proj4_pkg.SoftGripperState
            state of the gripper
        """
        
        self.current_state = msg

    def step_control(self, desired_angle):
        """
        Controls the fingers to the desired angle

        Parameters
        ----------
        desired_angle : int
            the desired angle to send both fingers to
        """

        ########## YOUR CODE HERE ##########
        left_pwm = 0
        right_pwm = 0
        ####################################
        self.send_gripper_cmd(left_pwm, right_pwm)

    def run(self):
        """
        Commands the fingers to linearly increase their angles for 10 seconds,
        then holds the end angle for another 10 seconds.
        """

        for i in range(args.start_angle, args.end_angle + 1):
            self.step_control(i)
            rospy.sleep(10.0 / (args.end_angle - args.start_angle))
        for _ in range(20):
            self.step_control(args.end_angle)
            rospy.sleep(0.5)
        self.shutdown()

    def shutdown(self):
        """
        Sets the gripper back to neutral when exiting
        """
        
        self.send_gripper_cmd(0, 0)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-start_angle", "-s", type=int, default=10, help=
        """Starting angle for the grasper."""
    )
    parser.add_argument("-end_angle", "-e", type=int, default=50, help=
        """Ending angle for the grasper."""
    )
    args, unknown = parser.parse_known_args()
    rospy.init_node("grasp_controller", anonymous=True)
    gc = GraspController()
    gc.run()