import numpy as np

class TurtlebotBarrier:
    def __init__(self, stateDimn, inputDimn, dynamics, observerEgo, observerObstacle, buffer):
        """
        Double integrator system Lyapunov function.
        Args:
            stateDimn (int): length of (entire) state vector
            inputDimn (int): length of input vector
            dynamics (Dynamics): dynamics object for the entire turtlebot system
            observerEgo (EgoObserver): observer object for the turtlebot we're deciding the input to
            observerObstacle (EgoObserver): observer object for an obstacle
        """
        self.stateDimn = stateDimn
        self.inputDimn = inputDimn
        self.dynamics = dynamics
        
        #parameters to store values
        self._vals = None 

        self._barrierPt = None
        self._buffer = buffer #barrier buffer
        self.observerEgo = observerEgo #store the system observer
        self.observerObstacle = observerObstacle #store an observer for the obstacle

        #define the radius of the turtlebot
        self.rt = 0.15
    
    def eval(self, u, t):
        """
        Evaluate the Euclidean distance to the barrier point.
        Args:
            u (input_dimn x 1 numpy array): input vector
        Returns:
            (List): cbf time derivatives
        """
        #get the position and velocity of the ego and the obstacle objects
        qe = self.observerEgo.get_state()

        #calculate qeDot from system dynamics (Not from observer ego)
        phi = qe[2, 0]
        qeDot = np.array([[np.cos(phi), 0], [np.sin(phi), 0], [0, 1]])@u

        #get the obstacle states from the observer
        qo = self.observerObstacle.get_state()
        qoDot = self.observerObstacle.get_vel()

        #evaluate the CBF
        h = (qe[0, 0] - qo[0, 0])**2 + (qe[1, 0] - qo[1, 0])**2 - (2*self.rt)**2

        #evaluate the derivative of the CBF
        hDot = 2*(qe[0, 0] - qo[0, 0])*((qeDot[0, 0] - qoDot[0, 0])) + 2*(qe[1, 0] - qo[1, 0])*((qeDot[1, 0] - qoDot[1, 0]))
        
        #return the two derivatives and the barrier function
        self._vals = [h, hDot]
        return self._vals

    def get(self):  
        """
        Retrieves stored function and derivative values
        Returns:
        [..., vDDot, vDot, v] ((self.dynamics.relDegree + 1, 1) numpy Array): dynamics.relDegree lyapunov/Barrier time derivs, Descending order
        """
        return self._vals

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


    def calc_rotation(self, qe):
        """
        Calculates the rotation matrix Rse associated with a particular state vector
        Inputs:
            qe ((3x1) NumPy Array): state vector of the ego turtlebot
        Returns:
            Rse ((3x3) NumPy Array): rotation matrix from the ego frame to spatial frame
        """
        phi = qe[2, 0]
        Rse = np.array([[np.cos(phi), -np.sin(phi), 0], [np.sin(phi), np.cos(phi), 0], [0, 0, 1]])
        return Rse

    def ptcloud_to_spatial(self):
        """
        Transforms the pointcloud from the ego turtlebot frame in which it was taken
        to the spatial frame. 
        Inputs:
            ptcloudDict (Dictionary): {'ptcloud', 'stateVec'} dictionary
        Returns:
            ptcloudSpatial ((3xN) NumPy Array): pointcloud transformed to the world frame
        """
        #calculate rotation and position
        Rse = self.calc_rotation(self.ptcloudDict['stateVec'])
        qe = self.ptcloudDict['stateVec']
        pse = np.array([[qe[0, 0], qe[1, 0], 0]]).T

        #transform ptcloud using pse, Rse
        return Rse @ self.ptcloudDict['ptcloud'] + pse

    def eval(self, u, t):
        """
        Evaluate the CBF and its derivatives. 
        Inputs:
            u ((2x1) NumPy Array): the z input to the nonlinear system.
        Returns:
            [h, hDot] (Python List of floats): barrier function and its derivatives
        """
        #get the position and velocity of the ego and the obstacle objects
        qe = self.observerEgo.get_state().reshape((3, 1))

        #calculate qeDot from system dynamics (Not from observer ego)
        phi = qe[2, 0]
        qeDot = np.array([[np.cos(phi), 0], [np.sin(phi), 0], [0, 1]])@u

        #get the pointcloud in the spatial frame
        ptcloudSpatial = self.ptcloud_to_spatial()

        #get the closest point in the pointcloud as qo for barrier computation
        diffmatrix = ptcloudSpatial - qe
        normMatrix = np.linalg.norm(diffmatrix, axis = 0) #get the norms of the columns
        colMin = np.argmin(normMatrix) #find the closest column
        qo = ptcloudSpatial[:, colMin].reshape((3, 1)) #define the obstacle point from the closest column

        # print("qo: ", qo.T)

        #assume the obstacle velocities and accelerations are negligible
        qoDot = np.zeros((3, 1))
        zo = np.zeros((3, 1))

        #evaluate the CBF
        h = (qe[0, 0] - qo[0, 0])**2 + (qe[1, 0] - qo[1, 0])**2 - (2*self.rt)**2

        #evaluate the derivative of the CBF
        hDot = 2*(qe[0, 0] - qo[0, 0])*((qeDot[0, 0] - qoDot[0, 0])) + 2*(qe[1, 0] - qo[1, 0])*((qeDot[1, 0] - qoDot[1, 0]))

        #using the linearized dynamics, return the second derivative of the CBF
        # hDDot = 2*(qeDot[0, 0] - qoDot[0, 0])**2 + 2*(qe[0, 0] - qo[0, 0])*((u[0] - zo[0, 0])) + 2*(qeDot[1, 0] - qoDot[1, 0])**2 + 2*(qe[1, 0] - qo[1, 0])*((u[1] - zo[1, 0]))
        
        #return the one derivative and the barrier function
        self._vals = [h, hDot]
        # self._vals = [h, 0, 0]
        return self._vals

    def get(self):
        """
        Retreives stored function and derivative values
        Returns:
        [..., vDDot, vDot, v] ((self.dynamics.relDegree + 1, 1) numpy Array): dynamics.relDegree lyapunov/Barrier time derivs, Descending order
        """
        return self._vals