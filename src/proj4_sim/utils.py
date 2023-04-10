"""
Utilities file for running simulations. Includes functions to generate initial and goal states.
"""
import numpy as np


#NOTE: DEFINE CONSTANTS FOR INITIAL AND GOAL STATES HERE
#define circle radius
r = 2.5

#define circle center
xC, yC = 2.5, 2.5

#define an initial condition generator
def gen_init_cond(N):
    """
    Function to return initial condition for a system of N bots.
    Evenly spaces a set of turtlebots on a circle.
    """
    #generate a vector of zeros
    q0 = np.zeros((3*N, 1))

    #offset for assymetry to avoid deadlock
    offset = 0.17

    #find the initial condition vector for each turtlebot
    for i in range(N):
        #get x and y, place into the vector
        x0 = r*(np.cos(i/N * 2*np.pi + offset)) + xC
        y0 = r*(np.sin(i/N * 2*np.pi + offset)) + yC
        q0[3*i : 3*i + 3, 0] = np.array([x0, y0, 0])

    return q0

#define a goal state vector generator
def gen_goal_state(N):
    """
    Function to define a goal state vector for a system of N bots.
    Evenly spaces a set of goal states on a circle 180 degrees from the initial cond.
    """
    #generate a vector of zeros
    qD = np.zeros((3*N, 1))

    #find the initial condition vector for each turtlebot
    for i in range(N):
        #get x and y, place into the vector
        x0 = r*(np.cos(i/N * 2*np.pi + np.pi)) + xC
        y0 = r*(np.sin(i/N * 2*np.pi + np.pi)) + yC
        qD[3*i : 3*i + 3, 0] = np.array([x0, y0, 0])
    
    return qD