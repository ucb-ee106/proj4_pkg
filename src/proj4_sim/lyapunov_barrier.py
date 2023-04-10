import numpy as np

class LyapunovBarrier:
    """
    Skeleton class for Lyapunov/Barrier functions.
    Includes utilities to get Lyapunov/Barrier values and Lyapunov/Barrier derivatives
    """
    def __init__(self, stateDimn, inputDimn, dynamics):
        """
        Init function for a Lyapunov/Barrier object
        Args:
            stateDimn (int): length of state vector
            inputDimn (int): length of input vector
            dynamics (Dynamics): dynamics object
        """
        self.stateDimn = stateDimn
        self.inputDimn = inputDimn
        self.dynamics = dynamics
        
        #parameters to store values
        self._vals = None #stored derivatives + value of function (initialize as None)
    
    def eval(self, u, t):
        """
        Returns a list of derivatives (going until zeroth derivative) of the lyapunov/Barrier function.
        Args:
            u (numpy array, (input_dimn x 1)): current input vector to system
            t (float): current time in simulation
        Returns:
        [..., vDDot, vDot, v] ((self.dynamics.relDegree + 1, 1) numpy Array): dynamics.relDegree lyapunov/Barrier time derivs, Descending order
        """
        self._vals = np.zeros((self.dynamics.relDegree + 1, 1))
        return self._vals
    
    def get(self):
        """
        Retreives stored function and derivative values
        Returns:
        [..., vDDot, vDot, v] ((self.dynamics.relDegree + 1, 1) numpy Array): dynamics.relDegree lyapunov/Barrier time derivs, Descending order
        """
        return self._vals
    
"""
********************************
ADD YOUR BARRIER FUNCTIONS HERE
********************************
"""

class TurtlebotBarrier(LyapunovBarrier):
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
        super().__init__(stateDimn, inputDimn, dynamics)
        self._barrierPt = None
        self._buffer = buffer #barrier buffer
        self.observerEgo = observerEgo #store the system observer
        self.observerObstacle = observerObstacle #store an observer for the obstacle

        #define the radius of the turtlebot
        self.rt = 0.15
    
class TurtlebotBarrierDeadlock(TurtlebotBarrier):
    def __init__(self, stateDimn, inputDimn, dynamics, observerEgo, observerObstacle, buffer):
        """
        Turtlebot barrier function for deadlock resolution. This barrier function is relative degree 2
        with respect to the linearized dynamics q'Dot = Aq' + Bz -> these are used to compute the derivative.
        """
        #call the super init function
        super().__init__(stateDimn, inputDimn, dynamics, observerEgo, observerObstacle, buffer)

    def eval(self, u, t):
        """
        Evaluate the CBF and its derivatives. 
        Inputs:
            u ((2x1) NumPy Array): the z input to the linear system from feedback linearization
        Returns:
            [h, hDot, hDDot] (Python List of floats): barrier function and its derivatives along the trajectories of the linear system
        """
        #get the position and velocity of the ego and the obstacle objects
        qe = self.observerEgo.get_state()

        #calculate qeDot from the observer (Not from v input!)
        qeDot = self.observerEgo.get_vel()

        #get the obstacle states from the observer
        qo = self.observerObstacle.get_state()
        qoDot = self.observerObstacle.get_vel()
        zo = self.observerObstacle.get_z()

        """
        TODO: Your code here:

        Evaluate your control barrier function and its derivatives. Once you've
        evaluated its derivatives (ex: h, hDot, hDdot, ...) place them in the self._vals
        list provided at the bottom of this function.

        In this code, a subscript 'e' stands for ego - this is the turtlebot we wish to control.
        A subscript 'o' stands for obstacle - these are for the turtlebots we wish to avoid.
        
        All necessary states have been extracted above for you. You don't necessarily need to use all of these
        terms in your CBF and derivative calculations, but they may be helpful.

            u ((2x1) Casadi array) is the input to the system from a casadi optimization
            t (float) is the current time in the simulation
            qe ((3x1) NumPy array) is the current state vector of the turtlebot we wish to control
            qeDot ((3x1) NumPy array) is the current time derivative of the state vector of the turtlebot we wish to control
            qo ((3x1) NumPy array) is the current state vector of the obstacle turtlebot we wish to avoid
            qoDot ((3x1) NumPy array) is the derivative of the obstacle turtlebot's state vector [xDot, yDot, phiDot]
            zo ((2x1) NumPy Array) is the current input being sent to the obstacle turtlebot in the INNERMOST feedback linearizing step
        
        NOTE: The radius of the turtlebot is stored in self.rt - you may assume all turtlebots have the same radius.
        """

        h = ...
        hDot = ...
        hDDot = ...

        #TODO: calculate h, hDot, hDDot and place them in this list (h and its first and second time derivatives)
        self._vals = [h, hDot, hDDot]
        return self._vals
    
class TurtlebotBarrierVision(TurtlebotBarrierDeadlock):
    def __init__(self, stateDimn, inputDimn, dynamics, observerEgo, lidarEgo, buffer):
        """
        Turtlebot barrier function for vision-based CBF-QP. This barrier function is relative degree 2
        with respect to the linearized dynamics q'Dot = Aq' + Bz -> these are used to compute the derivative.
        """
        #store the input parameters
        self.stateDimn = stateDimn
        self.inputDimn = inputDimn
        self.dynamics = dynamics
        self.observerEgo = observerEgo
        self.lidarEgo = lidarEgo #Lidar Ego is a lidar object from the ego observer
        self._buffer = buffer

        #define the radius of the turtlebot
        self.rt = 0.15

    def eval(self, u, t):
        """
        Evaluate the CBF and its derivatives. 
        Inputs:
            u ((2x1) NumPy Array): the z input to the linearized system.
        Returns:
            [h, hDot, hDDot] (Python List of floats): barrier function and its derivatives
        """
        
        #get the position and velocity of the ego and the obstacle objects
        qe = self.observerEgo.get_state()

        #calculate qeDot from the observer (Not from v input!)
        qeDot = self.observerEgo.get_vel()

        #Call the lidar update step to recompute pointcloud
        ptcloudDict = self.lidarEgo.get_pointcloud(update = True)

        """
        TODO: Your code here:

        Evaluate your control barrier function and its derivatives. Once you've
        evaluated its derivatives (ex: h, hDot, hDdot, ...) place them in the self._vals
        list provided at the bottom of this function.

        In this code, a subscript 'e' stands for ego - this is the turtlebot we wish to control.
        
        All necessary states have been extracted above for you. You don't necessarily need to use all of these
        terms in your CBF and derivative calculations, but they may be helpful. You no longer have direct access 
        to the obstacle turtlebot states, and now have to rely purely on lidar data for obstacle avoidance.
        You'll have to make a few assumptions about the environment to apply CBF control in this case.

            u ((2x1) Casadi array) is the input to the system from a casadi optimization
            t (float) is the current time in the simulation
            qe ((3x1) NumPy array) is the current state vector of the turtlebot we wish to control
            qeDot ((3x1) NumPy array) is the current time derivative of the state vector of the turtlebot we wish to control
            ptcloudDict (Dictionary) is the dictionary containing the pointcloud and the state vector of the ego turtlebot
            at the instant the pointcloud was taken. The dictionary has the structure:
                ptcloudDict = {'ptcloud': (3xN NumPy Array), 'stateVec': (3x1 NumPy Array)}
            Where calling ptcloudDict['ptcloud'] returns the [x, y, z] pointcloud of the surroundings as taken by the lidar. 
            Each column of this pointcloud is a 3D point detected by the lidar within the frame of the ego turtlebot (the bot we wish to control).
        
        NOTE: You may assume that the z coordinate of each point in the pointcloud will always be zero.
        NOTE: The radius of the turtlebot is stored in self.rt - you may assume all turtlebots have the same radius.
        Hints: You may find it useful to define helper functions to calculate the rotation matrix of the turtlebot to the world frame and to
        transform the pointcloud to the world frame.
        """

        h = ...
        hDot = ...
        hDDot = ...
        
        #return the two derivatives and the barrier function
        self._vals = [h, hDot, hDDot]
        return self._vals


class BarrierManager:
    def __init__(self, N, stateDimn, inputDimn, dynamics, observer, buffer, dLock = False, lidarManager = None):
        """
        Class to organize barrier functionf or a system of N turtlebots.
        Initializes N-1 TurtlebotBarrier objects for each of the N turtlebots in the system.
        Provides utilities for returning the N-1 barrier functions and their derivatives for each turtlebot.
        Inputs:
            stateDimn (int): length of state vector
            inputDimn (int): length of input vector
            dynamics (Dynamics): dynamics object for the whole turtlebot system
            observer (ObserverManager): observer object for the whole turtlebot system
            dLock (boolean): apply deadlock version of CBF to the system
            lidarManager (LidarManager): include if wish to use the vision-based version of the CBF
        """
        #store the number of turtlebots in the system
        self.N = N

        #create a barrier function dictionary that stores the N - 1 barriers for each turtlebot
        self.barrierDict = {}

        #store the observer manager
        self.observerManager = observer

        #store the deadlock and vision options
        self.dLock = dLock
        self.lidarManager = lidarManager

        #create a set of N - 1 Barrier functions for that turtlebot
        for i in range(self.N):
            #create the N - 1 turtlebots - one for all indices other than i
            indexList = list(range(N))

            #remove the ego turtlebot index i from this list
            indexList.remove(i)

            #initialize an empty list to contain the barrier functions for the ego turtlebot
            egoBarrierList = []

            #get the observer corresponding to the ego turtlebot
            observerEgo = self.observerManager.get_observer_i(i)

            if self.lidarManager is not None:
                #get the lidar associated with turtlebot i (the EGO turtlebot)
                lidarEgo = self.lidarManager.get_lidar_i(i)

                #use the vision-based barrier function - only need one per turtlebot
                egoBarrierList.append(TurtlebotBarrierVision(stateDimn, inputDimn, dynamics, observerEgo, lidarEgo, buffer))
            else:
                #use the non vision-based barrier functions - create one for each obstacle
                #loop over the remaining indices and create a barrier function for each obstacle turtlebot
                for j in indexList:
                    #ego index is i, obstacle index is j
                    observerObstacle = self.observerManager.get_observer_i(j) #get the obstacle observer

                    #append a barrier function with the obstacle and ego observers
                    if not self.dLock and not self.vision:
                        #use the basic relative degree 1 barrier function
                        egoBarrierList.append(TurtlebotBarrier(stateDimn, inputDimn, dynamics, observerEgo, observerObstacle, buffer))                        
                    else:
                        #apply the deadlock version (r = 2) of the barrier function (not vision-based)
                        egoBarrierList.append(TurtlebotBarrierDeadlock(stateDimn, inputDimn, dynamics, observerEgo, observerObstacle, buffer))

            #store the ego barrier list in the barrier function dictionary for the robots
            self.barrierDict[i] = egoBarrierList

    def get_barrier_list_i(self, i):
        """
        Function to retrieve the list of barrier function objects corresponding to turtlebot i
        Inputs:
            i (int): index of turtlebot (zero-indexed)
        Returns:
            barrierList (TurtlebotBarrier List): List of TurtlebotBarrier objects corresponding to turtlebot i
        """
        return self.barrierDict[i]