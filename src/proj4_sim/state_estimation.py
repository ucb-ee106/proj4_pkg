import numpy as np

class StateObserver:
    def __init__(self, dynamics, mean = None, sd = None):
        """
        Init function for state observer

        Args:
            dynamics (Dynamics): Dynamics object instance
            mean (float, optional): Mean for gaussian noise. Defaults to None.
            sd (float, optional): standard deviation for gaussian noise. Defaults to None.
        """
        self.dynamics = dynamics
        self.stateDimn = dynamics.stateDimn
        self.inputDimn = dynamics.inputDimn
        self.mean = mean
        self.sd = sd
        
    def get_state(self):
        """
        Returns a potentially noisy observation of the system state
        """
        if self.mean or self.sd:
            #return an observation of the vector with noise
            return self.dynamics.get_state() + np.random.normal(self.mean, self.sd, (self.stateDimn, 1))
        return self.dynamics.get_state()
    
class EgoTurtlebotObserver(StateObserver):
    def __init__(self, dynamics, mean, sd, index):
        """
        Init function for a state observer for a single turtlebot within a system of N turtlebots
        Args:
            dynamics (Dynamics): Dynamics object for the entire turtlebot system
            mean (float): Mean for gaussian noise. Defaults to None.
            sd (float): standard deviation for gaussian noise. Defaults to None.
            index (Integer): index of the turtlebot in the system
        """
        #initialize the super class
        super().__init__(dynamics, mean, sd)

        #store the index of the turtlebot
        self.index = index
    
    def get_state(self):
        """
        Returns a potentially noisy measurement of the state vector of the ith turtlebot
        Returns:
            3x1 numpy array, observed state vector of the ith turtlebot in the system (zero indexed)
        """
        return super().get_state()[3*self.index : 3*self.index + 3].reshape((3, 1))
    
    def get_vel(self):
        """
        Returns a potentially noisy measurement of the derivative of the state vector of the ith turtlebot
        Inputs:
            None
        Returns:
            3x1 numpy array, observed derivative of the state vector of the ith turtlebot in the system (zero indexed)
        """
        #first, get the current input to the system of turtlebots
        u = self.dynamics.get_input()

        #now, get the noisy measurement of the entire state vector
        x = self.get_state()

        #to pass into the deriv function, augment x with zeros elsewhere
        x = np.vstack((np.zeros((self.index*3, 1)), x, np.zeros(((self.dynamics.N - 1 - self.index)*3, 1))))
        
        #calculate the derivative of the ith state vector using the noisy state measurement
        xDot = self.dynamics.deriv(x, u, 0) #pass in zero for the time (placeholder for time invar system)

        #slice out the derivative of the ith turtlebot and reshape
        return xDot[3*self.index : 3*self.index + 3].reshape((3, 1))
    
    def get_z(self):
        """
        Return the augmented input vector, z, of the system from the ith turtlebot.
        Inputs:
            None
        Returns:
            z (2x1 NumPy Array): augmented input vector of the ith turtlebot
        """
        #call the get z function from the system dynamics
        z = self.dynamics.get_z()

        #slice out the ith term
        return z[2*self.index : 2*self.index + 2].reshape((2, 1))
    
    
class ObserverManager:
    def __init__(self, dynamics, mean, sd):
        """
        Managerial class to manage the observers for a system of N turtlebots
        Args:
            dynamics (Dynamics): Dynamics object instance
            mean (float, optional): Mean for gaussian noise. Defaults to None.
            sd (float, optional): standard deviation for gaussian noise. Defaults to None.
        """
        #store the input parameters
        self.dynamics = dynamics
        self.mean = mean
        self.sd = sd

        #create an observer dictionary storing N observer instances
        self.observerDict = {}

        #create N observer objects
        for i in range(self.dynamics.N):
            #create an observer with index i
            self.observerDict[i] = EgoTurtlebotObserver(dynamics, mean, sd, i)

    def get_observer_i(self, i):
        """
        Function to retrieve the ith observer object for the turtlebot
        Inputs:
            i (integet): index of the turtlebot whose observer we'd like to retrieve
        """
        return self.observerDict[i]
    
    def get_state(self):
        """
        Returns a potentially noisy observation of the *entire* system state (vector for all N bots)
        """
        #get each individual observer state
        xHatList = []
        for i in range(self.dynamics.N):
            #call get state from the ith observer
            xHatList.append(self.get_observer_i(i).get_state())

        #vstack the individual observer states
        return np.vstack(xHatList)
    

class EgoLidar:
    def __init__(self, index, observerManager, mean = None, sd = None):
        """
        Lidar for an ego turtlebot. Returns a pointcloud containing
        other turtlebots in the environment.

        Args:
            index (int) : index of the ego turtlebot (zero-indexed from the first turtlebots)
            observerManager (ObserverManager) : Manager object for state estimation
            mean (float): mean for measurement noise
            sd (float): standard deviation for measurement noise
        """
        #store input parameters
        self.index = index
        self.observerManager = observerManager
        self.mean = mean
        self.sd = sd

        #store an attribute for the pointcloud
        self.numPts = 20 #number of points per bot for depth camera observation
        self._ptcloudData = {} #pointcloud attribute (initialize far away)

    def calc_orientation(self):
        """
        Function to calculate the orientation of the turtlebot WRT the world frame.
        Args:
            None
        Returns:
            Rse: rotation matrix from ego frame to world frame
        """
        qo = (self.observerManager.get_observer_i(self.index)).get_state()
        phi = qo[2, 0]
        Rse = np.array([[np.cos(phi), -np.sin(phi), 0], [np.sin(phi), np.cos(phi), 0], [0, 0, 1]])
        return Rse

    def calc_ptcloud(self):
        """
        Function to compute the pointcloud of the environment from the ego turtlebot frame.
        Args:
            None
        Returns:
            ptcloud (3 x N NumPy Array): pointcloud containing (x, y, z) coordinates of obstacles within ego frame.
        """
        #get the ego state vector and orientation
        qEgo = (self.observerManager.get_observer_i(self.index)).get_state()
        pse = np.array([[qEgo[0, 0], qEgo[1, 0], 0]]).T #ego XYZ position
        Rse = self.calc_orientation() #ego rotation matrix to spatial frame

        #get a list of observers for all obstacle turtlebots
        obsIndexList = list(self.observerManager.observerDict.keys()) #get a list of all turtlebot indices
        obsIndexList.remove(self.index) #remove the ego index

        #define an array of angles to generate points over
        thetaArr = np.linspace(0, 2*np.pi, self.numPts).tolist()

        #define an empty pointcloud - should be 3 x numPts
        ptcloud = np.zeros((3, len(obsIndexList) * self.numPts))
        
        #define a number of iterations
        j = 0

        #iterate over all obstacle turtlebots
        for i in obsIndexList:
            #get the state vector and radius of the ith turtlebot
            qo = (self.observerManager.get_observer_i(i)).get_state()
            rt = self.observerManager.dynamics.rTurtlebot

            #calculate the points on the circle - all z values are zero
            xList = [rt*np.cos(theta) + qo[0, 0] for theta in thetaArr]
            yList = [rt*np.sin(theta) + qo[1, 0] for theta in thetaArr]
            zList = [0 for theta in thetaArr]

            #put the points in a numpy array
            ptcloudI = np.array([xList, yList, zList])

            #transform the points into the ego frame
            ptcloudIego = (np.linalg.inv(Rse)@(ptcloudI - pse))

            #store in the ptcloud
            ptcloud[:, j * self.numPts : j*self.numPts + self.numPts] = ptcloudIego

            #increment number of iterations
            j += 1
            
        #store both the pointcloud and the ego state vector at the time the pointcloud is taken
        self._ptcloudData["ptcloud"] = ptcloud
        self._ptcloudData["stateVec"] = qEgo
        return self._ptcloudData
        
    def get_pointcloud(self, update = True):
        """
        Returns the pointcloud dictionary from the class attribute 
        Args:
            update: whether or not to recalculate the pointcloud
        Returns:
            Dict: dictionary of pointcloud points and state vector at time of capture
        """
        #first, calculate the pointcloud
        if update:
            self.calc_ptcloud()
        return self._ptcloudData
    
class LidarManager:
    def __init__(self, observerManager, mean, sd):
        """
        Managerial class to manage the observers for a system of N turtlebots
        Args:
            observerManager (ObserverManager): ObserverManager object instance
            mean (float, optional): Mean for gaussian noise. Defaults to None.
            sd (float, optional): standard deviation for gaussian noise. Defaults to None.
        """
        #store the input parameters
        self.observerManager = observerManager
        self.mean = mean
        self.sd = sd

        #create an observer dictionary storing N observer instances
        self.lidarDict = {}

        #create N lidar objects - one for each turtlebot
        for i in range(self.observerManager.dynamics.N):
            #create an observer with index i
            self.lidarDict[i] = EgoLidar(i, self.observerManager, mean, sd)

    def get_lidar_i(self, i):
        """
        Function to retrieve the ith observer object for the turtlebot
        Inputs:
            i (integer): index of the turtlebot whose observer we'd like to retrieve
        """
        return self.lidarDict[i]