#!/usr/bin/env python
"""
User interface for the soft gripper
Author: Chris Correa and Josephine Koe
"""
import numpy as np
import pandas as pd
import sys
import os
import traceback

import rospy
import argparse
import time

from proj4_pkg.msg import SoftGripperCmd, SoftGripperState
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression

class SoftGripperUserInterface():
    def __init__(self, filename=""):
        """
        User interface for the soft gripper

        Parameters
        ----------
        filename : string
            file name to save data to or display data from
        """

        # Set up ROS node
        self.command_pub = rospy.Publisher("soft_gripper_cmd", SoftGripperCmd, queue_size=10)
        rospy.sleep(1)
        rospy.Subscriber("soft_gripper_state", SoftGripperState, self.state_listener)
        rospy.on_shutdown(self.shutdown)
        self.rate = rospy.Rate(100)

        # Set up file
        self.filedir = os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))), "data/")

        if filename:
            self.filename = self.filedir + filename + ".csv"
        else:
            self.filename = None

        # Set up states
        self.column_names = [
            "time", 
            "left_pwm", 
            "left_pressure",
            "left_flex", 
            "left_angle",
            "right_pwm", 
            "right_pressure", 
            "right_flex",
            "right_angle"
        ]
        self.states = pd.DataFrame(columns=self.column_names)
        if self.filename:
            if os.path.exists(self.filename):
                self.states = pd.read_csv(self.filename)
            elif not os.path.exists(self.filedir):
                os.makedirs(self.filedir)
    
        self.left_angle = None
        self.right_angle = None
        self.max_pwm = 100

    def send_gripper_cmd(self, left=-1, right=-1):
        """
        Sends PWM commands to the soft gripper

        Parameters
        ----------
        left : int
            PWM value for the left gripper
        right : int
            PWM value for the right gripper
        """

        self.command_pub.publish(SoftGripperCmd(min(self.max_pwm, int(left)), min(self.max_pwm, int(right))))

    def send_cmd_from_terminal(self):
        """
        Prompts the user to input PWM commands and sends the values to the soft gripper

        Returns
        -------
        True if the user wants to quit, else None
        """

        while not rospy.is_shutdown():
            user_input = raw_input("Left PWM, Right PWM: ").lower()
            if user_input == "q":
                return True
            try:
                left, right = user_input.split(",")
                self.send_gripper_cmd(left, right)
                break
            except:
                print("Could not parse:", user_input)
                traceback.print_exc(file=sys.stdout)

    def record_finger(self, finger):
        """
        Generates an angle data point for one finger

        Parameters
        ----------
        finger : string
            the finger to record ("left" or "right")

        Returns
        -------
        True if the user wants to quit, else None
        """

        while not rospy.is_shutdown():
            user_input = raw_input("%s PWM: " % finger.capitalize()).lower()
            if user_input == "q":
                return True
            try:
                if finger == "left":
                    self.send_gripper_cmd(left=user_input)
                else:
                    self.send_gripper_cmd(right=user_input)
                break
            except:
                print("Could not parse:", user_input)
                traceback.print_exc(file=sys.stdout)

        while not rospy.is_shutdown():
            user_input = raw_input("%s steady state angle: " % finger.capitalize()).lower()
            if user_input == "q":
                return True
            try:
                if finger == "left":
                    self.left_angle = float(user_input)
                else:
                    self.right_angle = float(user_input)
                break
            except:
                print("Could not parse:", user_input)
                traceback.print_exc(file=sys.stdout)

    def state_listener(self, msg):
        """
        Records states from the soft_gripper_state topic

        Parameters
        ----------
        msg : proj4_pkg.SoftGripperState
            state of the gripper
        """

        if self.filename and args.mode != "display":
            self.states.loc[len(self.states)] = [
                msg.time,
                msg.left_pwm, 
                msg.left_pressure, 
                msg.left_flex, 
                self.left_angle,
                msg.right_pwm, 
                msg.right_pressure, 
                msg.right_flex,
                self.right_angle
            ]
        self.left_angle = None
        self.right_angle = None

    def save_data(self):
        """
        Saves data to file
        """

        if self.filename:
            self.states.to_csv(self.filename, index=False)
            print("Saved data to %s" % self.filename)

    def play(self):
        """
        Listens for gripper commands and publishes to ROS
        """

        print("################################################################################\n" + \
              "Enter in integer PWM values for the fingers in the format left_pwm, right_pwm.\n" + \
              "If you want a finger's value to remain unchanged, enter in -1 for its value.\n\n" + \
              "Enter the letter 'q' to quit.\n" + \
              "################################################################################\n")
        while not rospy.is_shutdown():
            if self.send_cmd_from_terminal():
                break
            self.rate.sleep()
        self.save_data()

    def record_angles(self):
        """
        Records angle data for the left finger, then the right finger
        """

        if self.filename:
            print("################################################################################\n" + \
                  "Record angle data for the left finger, then the right finger. For each finger:\n\n" + \
                  "First, enter in integer PWM values for the finger you are recording.\n" + \
                  "Wait until the finger stabilized, then enter in the observed steady state angle for the finger at that PWM value.\n" + \
                  "Repeat this process until you have enough data points for the finger.\n\n" + \
                  "Enter the letter 'q' to move on to the next finger, or again to quit.\n" + \
                  "################################################################################\n")

            print("Recording the left finger\n")
            while not rospy.is_shutdown():
                if self.record_finger("left"):
                    break
                self.rate.sleep()

            print("\nRecording the right finger\n")
            while not rospy.is_shutdown():
                if self.record_finger("right"):
                    break
                self.rate.sleep()
            self.save_data()
        else:
            print("Please use a command line argument to provide a file name to save the data to.")

    def display_data(self):
        """
        Displays data as plots
        """

        if self.filename and (os.path.exists(self.filename)):
            # Prepare left finger angle data
            left_data = self.states[self.states.left_angle.notnull()]
            left_pwms = np.array(left_data["left_pwm"])
            left_flexs = np.array(left_data["left_flex"])
            left_pressures = np.array(left_data["left_pressure"])
            left_angles = np.array(left_data["left_angle"])

            # Prepare right finger angle data
            right_data = self.states[self.states.right_angle.notnull()]
            right_pwms = np.array(right_data["right_pwm"])
            right_flexs = np.array(right_data["right_flex"])
            right_pressures = np.array(right_data["right_pressure"])
            right_angles = np.array(right_data["right_angle"])

            # Plot angle data
            fig, [[left_pwm_plot, right_pwm_plot], 
                  [left_pressure_plot, right_pressure_plot],
                  [left_flex_plot, right_flex_plot]] = plt.subplots(3, 2, figsize=(15,15))

            #################### YOU MAY EDIT THIS SECTION ####################
            self.make_angle_subplot(left_pwm_plot, 
                              left_angles, 
                              left_pwms, 
                              "left",
                              "angle",
                              "PWM value",
                              self.log_regression)
            self.make_angle_subplot(right_pwm_plot, 
                              right_angles, 
                              right_pwms, 
                              "right",
                              "angle",
                              "PWM value",
                              self.log_regression)
            self.make_angle_subplot(left_pressure_plot, 
                              left_angles, 
                              left_pressures, 
                              "left",
                              "angle",
                              "pressure value",
                              self.linear_regression)
            self.make_angle_subplot(right_pressure_plot, 
                              right_angles, 
                              right_pressures, 
                              "right",
                              "angle",
                              "pressure value",
                              self.linear_regression)
            self.make_angle_subplot(left_flex_plot, 
                              left_angles, 
                              left_flexs, 
                              "left",
                              "angle",
                              "flex value",
                              self.linear_regression)
            self.make_angle_subplot(right_flex_plot, 
                              right_angles, 
                              right_flexs, 
                              "right",
                              "angle",
                              "flex value",
                              self.linear_regression)
            ###################################################################

            # Display plots
            plt.show()
        else:
            print("Please use a command line argument to provide a valid file name to retrieve data from.")

    def make_angle_subplot(self, ax, x, y, finger, xlabel, ylabel, model=None):
        """
        Plots the data with a model on a subplot

        Parameters
        ----------
        ax : matplotlib.AxesSubplot
            subplot to diplay data on
        x : nx np.ndarray
            data for x axis
        y : nx np.ndarray
            data for y axis
        finger : string
            "left" or "right"
        xlabel : string
            label for x axis
        ylabel : string
            label for y axis
        model : method
            function for modeling the data
        """

        # Set plot title and axes
        title = finger.capitalize() + " " + xlabel + " vs. " + ylabel
        ax.set_title(title)
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        # Plot data
        ax.scatter(x, y)

        if model:
            # Calculate model
            pred_y, label = model(x, y)
            # Plot predicted values
            sort_idxs = np.argsort(x)
            ax.plot(x[sort_idxs], pred_y[sort_idxs], "r-", label=label)
            print("%s: %s" % (title, label))
            # Make a legend for the plot
            ax.legend()

    def linear_regression(self, x, y):
        """
        Calculates a linear regression for x and y

        Parameters
        ----------
        x : nx np.ndarray
            data for x axis
        y : nx np.ndarray
            data for y axis

        Returns
        -------
        pred_y : nx np.ndarray
            model predictions corresponding to the x values
        label : string
            label for plot legend
        """
        m, b = np.polyfit(x, y, 1)
        pred_y = m * x + np.tile(b, len(x))
        return pred_y, "y = %.3fx + %.3f" % (m, b)

    def log_regression(self, x, y):
        """
        Calculates a logarithmic regression for x and y

        Parameters
        ----------
        x : nx np.ndarray
            data for x axis
        y : nx np.ndarray
            data for y axis

        Returns
        -------
        pred_y : nx np.ndarray
            model predictions corresponding to the x values
        label : string
            label for plot legend
        """
        log_x = np.log(x + 1)
        m, b = np.polyfit(log_x, y, 1)
        pred_y = m * log_x + np.tile(b, len(x))
        return pred_y, "y = %.3f ln(x + 1) + %.3f" % (m, b)

    def shutdown(self):
        """
        Sets the gripper back to neutral when exiting
        """

        self.send_gripper_cmd(0, 0)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-mode", "-m", type=str, default="play", help=
        """Mode for using the script. Options: play, record_angles, display_data. Default: play."""
    )
    parser.add_argument("-filename", "-f", type=str, default="", help=
        """File name (without the .csv extension) to save data to. An empty user_input will not save the data."""
    )
    args, unknown = parser.parse_known_args()
    rospy.init_node("soft_gripper_user_interface", anonymous=True)
    sgi = SoftGripperUserInterface(args.filename)
    if args.mode == "play":
        sgi.play()
    elif args.mode == "record_angles":
        sgi.record_angles()
    elif args.mode == "display_data":
        sgi.display_data()
