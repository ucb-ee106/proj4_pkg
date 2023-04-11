import numpy as np

"""
Authors: Han Nguyen, Massimiliano de Sa, Spring 2023.
"""

class TurtlebotBarrierVision:
    def __init__(self, observerEgo):
        """
        Turtlebot barrier function for vision-based CBF-QP. This barrier function is relative degree ONE (different to sim version!!)
        with respect to the linearized dynamics q'Dot = Aq' + Bz -> these are used to compute the derivative.
        """
        #store the input parameters
        self.observerEgo = observerEgo
        self.ptcloudDict = {'ptcloud': np.zeros((3, 1)), 'stateVec': np.zeros((3, 1))}

        #define the radius of the turtlebot
        self.rt = 0.15

    def update_pointcloud(self, state, pointcloud):
        self.ptcloudDict["ptcloud"] = pointcloud
        self.ptcloudDict['stateVec'] = state.reshape((3, 1))

    def eval(self, u, t):
        """
        Evaluate the CBF and its derivatives. 
        Inputs:
            u ((2x1) NumPy Array): the u input to the nonlinear system. (NOT the z input)
        Returns:
            [h, hDot] (Python List of floats): barrier function and its derivatives
        """
        #get the position and velocity of the ego and the obstacle objects
        qe = self.observerEgo.get_state().reshape((3, 1))

        #get the pointcloud dictionary object
        ptcloudDict = self.ptcloudDict

        """
        TODO: Your code here
        Compute the value of the vision-based barrier function and its first time derivative.
        The first time derivative should now be computed along the trajectories of the nonlinear
        turtlebot dynamics, rather than the linear dynamics. This means that you'll only have to 
        take one derivative of h rather than two to get an input constraint.

        The pointcloud dictionary and state vector of the system have been extracted above for you.
        Recall that you can extract the pointcloud in the turtlebot frame and the state vector at the time
        the pointcloud was taken using:
            self.ptcloudDict['ptcloud']
            self.ptcloudDict['stateVec']

        Hint: define helper functions to compute the orientation of the turtlebot and transform the pointcloud.
        """

        h = ...
        hDot = ...

        #return the one derivative and the barrier function
        self._vals = [h, hDot]
        return self._vals

    def get(self):
        """
        Retreives stored function and derivative values
        Returns:
        [..., vDDot, vDot, v] ((self.dynamics.relDegree + 1, 1) numpy Array): dynamics.relDegree lyapunov/Barrier time derivs, Descending order
        """
        return self._vals