#!/usr/bin/env python
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

#Import the rospy package. For an import to work, it must be specified
#in both the package manifest AND the Python file in which it is used.
import rospy
import numpy as np

from state_estimation import StateObserver, EgoTurtlebotObserver
from trajectory import Trajectory
from controller import Controller, TurtlebotFBLin, TurtlebotCBFQP
from lidar import Lidar
from lyapunov_barrier import TurtlebotBarrierVision

#Define the method which contains the main functionality of the node.
def task_controller():
  """
  Controls a turtlebot whose position is denoted by turtlebot_frame,
  to go to a position denoted by target_frame
  """

  # Initialization Time
  start_time = rospy.get_time()
  frequency = 50
  r = rospy.Rate(frequency) # 10hz

  # Observer
  observer = EgoTurtlebotObserver()

  # Trajectory
  start_position = np.array([0, 0])
  end_position = np.array([1,0])
  time_duration = 0.1
  trajectory = Trajectory(start_position, end_position, time_duration)

  # Lidar
  lidar = Lidar()

  # Barrier
  barrier = TurtlebotBarrierVision(observer)
  barriers = [barrier] #put in list for class structure

  # Controller
  v_bounds = np.array([-0.10, 0.22])
  w_bounds = np.array([-2.84, 2.84])
  uBounds = np.vstack((v_bounds, w_bounds))
  controller = TurtlebotFBLin(observer, trajectory, frequency, uBounds)
  # controller = TurtlebotCBFQP(observer, barriers, trajectory, frequency, uBounds)

  # Loop until the node is killed with Ctrl-C
  while not rospy.is_shutdown():
    t = rospy.get_time() - start_time
    barrier.update_pointcloud(observer.get_state(), lidar.get_pointcloud())
    controller.eval_input(t)
    controller.apply_input()
    r.sleep()

# This is Python's sytax for a main() method, which is run by default
# when exectued in the shell
if __name__ == '__main__':
  # Check if the node has received a signal to shut down
  # If not, run the talker method

  #Run this program as a new node in the ROS computation graph 
  #called /turtlebot_controller.
  rospy.init_node('turtlebot_controller', anonymous=True)

  try:
    task_controller()
  except rospy.ROSInterruptException:
    pass