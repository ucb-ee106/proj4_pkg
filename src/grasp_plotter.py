#!/usr/bin/env python
"""
Grasp Plotter
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
import rosbag

from proj4_pkg.msg import SoftGripperCmd, SoftGripperState
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class GraspPlotter():
    def __init__(self, filename=""):
        """
        Grasp Plotter

        Parameters
        ----------
        filename : string
            name of file to save data to
        """

        # Set up ROS node
        rospy.sleep(1)
        rospy.Subscriber('soft_gripper_cmd', SoftGripperCmd, self.update_cmd)
        rospy.Subscriber("visualization_marker", Marker, self.update_cube_pose)
        rospy.Subscriber("usb_cam/image_raw", Image, self.write_to_bag)
        rospy.on_shutdown(self.shutdown)
        self.rate = rospy.Rate(100)
    
        # Set up files
        self.filedir = os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))), "data/")

        if filename:
            self.bag_filename = self.filedir + filename + ".bag"
            self.bag = rosbag.Bag(self.bag_filename, 'w')
            self.img_filename = self.filedir + filename + ".png"
        else:
            self.bag_filename = None
            self.bag = None
            print("Please use a command line argument to provide a file name to save the data to.")
            self.shutdown()

        # Bounds for plotting
        self.time_bounds = [0, 25]
        self.height_bounds = [-0.01, 0.02]
        self.height_goal = 0.005
        self.angle_bounds = [-0.4, 0.4]
        self.angle_goal = 0.2
        self.pwm_bounds = [0, 100]

        # Data storage
        self.cube_times = []
        self.cube_heights = []
        self.cube_angles = []
        self.pwm_times = []
        self.left_pwms = []
        self.right_pwms = []

        # Start plotting
        self.start_time = rospy.get_time()
        self.set_up_plots()

    def write_to_bag(self, msg):
        """
        Writes the image from the camera to the bag file

        Parameters
        ----------
        msg : sensor_msgs.Image
            image from camera
        """

        try:
            self.bag.write('usb_cam/image_raw', msg)
        except:
            pass

    def update_cube_pose(self, msg):
        """
        Saves the state of the cube

        Parameters
        ----------
        msg : visualization_msgs.Marker
            AR marker state
        """
        
        # if the AR marker is 0
        if msg.id == 0:
            # if this is the first time the marker has been seen
            if not hasattr(self, 'init_cube_pose'):
                # save the initial pose and yaw angle
                self.init_cube_pose = msg.pose
                self.init_cube_angle = self.calculate_yaw(self.init_cube_pose)
            # else if plotting has started
            elif hasattr(self, 'start_time'):
                # add the cube height and angle differences to the data
                self.cube_heights.append(self.init_cube_pose.position.y - msg.pose.position.y)
                current_yaw = self.calculate_yaw(msg.pose)
                diff = current_yaw - self.init_cube_angle
                centered_diff = np.mod(diff + np.pi, 2 * np.pi) - np.pi 
                self.cube_angles.append(centered_diff)
                self.cube_times.append(rospy.get_time() - self.start_time)

    def update_cmd(self, msg):
        """
        Saves the command to the gripper

        Parameters
        ----------
        msg : proj4_pkg.SoftGripperCmd
            commands to the gripper
        """
        
        if hasattr(self, 'start_time'):
            self.left_pwms.append(msg.left)
            self.right_pwms.append(msg.right)
            self.pwm_times.append(rospy.get_time() - self.start_time)

    def calculate_yaw(self, pose):
        """
        Calculates the yaw angle of the pose in radians

        Parameters
        ----------
        pose : geometry_msgs.Pose
            pose to calculate the yaw angle of

        Returns
        -------
        float : yaw angle of the pose
        """

        q = pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return np.arctan2(siny_cosp, cosy_cosp)

    def set_up_plots(self):
        """
        Sets up the animation
        """

        # make the plots
        self.fig, [self.height_plot, self.angle_plot, self.pwm_plot] = plt.subplots(3, 1, figsize=(15,15))

        # draw initial lines (with goals)
        self.height_line, = self.height_plot.plot([], [], '-')
        self.height_plot.plot(self.time_bounds, [self.height_goal, self.height_goal], 'r-')
        self.angle_line, = self.angle_plot.plot([], [], '-')
        self.angle_plot.plot(self.time_bounds, [self.angle_goal, self.angle_goal], 'r-')
        self.angle_plot.plot(self.time_bounds, [-self.angle_goal, -self.angle_goal], 'r-')
        self.left_pwm_line, = self.pwm_plot.plot([], [], '-', label="Left")
        self.right_pwm_line, = self.pwm_plot.plot([], [], '-', label="Right")

        # set bounds and labels for axes
        self.height_plot.set_xlim(self.time_bounds[0], self.time_bounds[1])
        self.height_plot.set_xlabel("Time")
        self.height_plot.set_ylim(self.height_bounds[0], self.height_bounds[1])
        self.height_plot.set_ylabel("Cube height displacement (m)")
        self.angle_plot.set_xlim(self.time_bounds[0], self.time_bounds[1])
        self.angle_plot.set_xlabel("Time")
        self.angle_plot.set_ylim(self.angle_bounds[0], self.angle_bounds[1])
        self.angle_plot.set_ylabel("Cube angle (degrees)")
        self.pwm_plot.set_xlim(self.time_bounds[0], self.time_bounds[1])
        self.pwm_plot.set_xlabel("Time")
        self.pwm_plot.set_ylim(self.pwm_bounds[0], self.pwm_bounds[1])
        self.pwm_plot.set_ylabel("PWM value")
        self.pwm_plot.legend()

        # start animation
        self.animation = FuncAnimation(self.fig, self.update_animation, interval=100)
        plt.ion()
        plt.show(block=True)

        # save the image
        if hasattr(self, 'img_filename') and self.img_filename:
            self.fig.savefig(self.img_filename)

    def update_animation(self, frame):
        """
        Updates the animation

        Parameters
        ----------
        frame : the frame of the animation

        Returns
        -------
        the updated lines of the animation
        """

        data_len = min(len(self.cube_times), len(self.cube_heights))
        self.height_line.set_data(self.cube_times[:data_len], self.cube_heights[:data_len])
        data_len = min(len(self.cube_times), len(self.cube_angles))
        self.angle_line.set_data(self.cube_times[:data_len], self.cube_angles[:data_len])
        data_len = min(len(self.pwm_times), len(self.left_pwms))
        self.left_pwm_line.set_data(self.pwm_times[:data_len], self.left_pwms[:data_len])
        data_len = min(len(self.pwm_times), len(self.right_pwms))
        self.right_pwm_line.set_data(self.pwm_times[:data_len], self.right_pwms[:data_len])   
        return self.height_line, self.angle_line, self.left_pwm_line, self.right_pwm_line

    def shutdown(self):
        """
        Sets the gripper back to neutral when exiting
        """

        if self.bag:
            self.bag.close()
            print("Saved data to %s" % self.bag_filename)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-filename", "-f", type=str, default="", help=
        """File name (without the .bag extension) to save data to. An empty user_input will not save the data."""
    )
    args, unknown = parser.parse_known_args()
    rospy.init_node("grasp_plotter", anonymous=True)
    gt = GraspPlotter(args.filename)