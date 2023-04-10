import numpy as np
import casadi as ca
from state_estimation import *

"""
File containing controllers 
"""
class Controller:
    def __init__(self, observer, lyapunov = None, trajectory = None, obstacleQueue = None, uBounds = None):
        """
        Skeleton class for feedback controllers
        Args:
            dynamics (Dynamics): system Dynamics object
            observer (Observer): state observer object
            lyapunov (LyapunovBarrier): lyapunov functions, LyapunovBarrier object
            trajectory (Trajectory): trajectory for the controller to track (could just be a constant point!)
            obstacleQueue (ObstacleQueue): ObstacleQueue object, stores all barriers for the system to avoid
            uBounds ((Dynamics.inputDimn x 2) numpy array): minimum and maximum input values to the system
        """
        #store input parameters
        self.observer = observer
        self.lyapunov = lyapunov
        self.trajectory = trajectory
        self.obstacleQueue = obstacleQueue
        self.uBounds = uBounds
        
        #store input
        self._u = None
    
    def eval_input(self, t):
        """
        Solve for and return control input
        Inputs:
            t (float): time in simulation
        Returns:
            u ((Dynamics.inputDimn x 1)): input vector, as determined by controller
        """
        self._u = np.zeros((self.observer.inputDimn, 1))
        return self._u
    
    def get_input(self):
        """
        Retrieves input stored in class parameter
        Returns:
            self._u: most recent input stored in class paramter
        """
        return self._u
    
class TurtlebotTest:
    def __init__(self, observer):
        """
        Test class for a system of turtlebot controllers.
        Simply drives each turtlebot straight (set u1 = 1).
        """
        #store observer
        self.observer = observer
    
    def eval_input(self, t):
        """
        Solves for the control input to turtlebot i using a CBF-QP controller
        Inputs:
            t (float): current time in simulation
        """
        #set velocity input to 1, omega to zero (simple test input)
        self._u = np.array([[1, 0.1]]).T

    def get_input(self):
        """
        Retrieves input stored in class parameter
        Returns:
            self._u: most recent input stored in class paramter
        """
        #call the observer to test
        q = self.observer.get_state()
        return self._u
    
class TurtlebotFBLin:
    def __init__(self, observer, trajectory):
        """
        Class for a feedback linearizing controller for a single turtlebot within a larger system
        Args:
            observer (Observer): state observer object
            traj (Trajectory): trajectory object
        """
        #store input parameters
        self.observer = observer
        self.trajectory = trajectory

        #store the time step dt for integration
        self.dt = 1/50 #from the control frequency in environment.py

        #store an initial value for the vDot integral
        self.vDotInt = 0

    def eval_z_input(self, t):
        """
        Solve for the input z to the feedback linearized system.
        Use linear tracking control techniques to accomplish this.
        """
        #get the state of turtlebot i
        q = self.observer.get_state()

        #get the derivative of q of turtlebot i
        qDot = self.observer.get_vel()

        #get the trajectory states
        xD, vD, aD = self.trajectory.get_state(t)

        #form the augmented state vector.
        qPrime = np.array([q[0, 0], q[1, 0], qDot[0, 0], qDot[1, 0]]).T

        #form the augmented desired state vector
        qPrimeDes = np.array([xD[0, 0], xD[1, 0], vD[0, 0], vD[1, 0]]).T

        #find a control input z to the augmented system
        k1 = 4 #pick k1, k2 s.t. eddot + k2 edot + k1 e = 0 has stable soln
        k2 = 4
        e = np.array([[xD[0, 0], xD[1, 0]]]).T - np.array([[q[0, 0], q[1, 0]]]).T
        eDot = np.array([[vD[0, 0], vD[1, 0]]]).T - np.array([[qDot[0, 0], qDot[1, 0]]]).T
        z = aD[0:2].reshape((2, 1)) + k2*eDot + k1 * e

        #return the z input
        return z
    
    def eval_w_input(self, t, z):
        """
        Solve for the w input to the system
        Inputs:
            t (float): current time in the system
            z ((2x1) NumPy Array): z input to the system
        """
        #get the current phi
        phi = self.observer.get_state()[2, 0]

        #get the (xdot, ydot) velocity
        qDot = self.observer.get_vel()[0:2]
        v = np.linalg.norm(qDot)

        #first, eval A(q)
        Aq = np.array([[np.cos(phi), -v*np.sin(phi)], 
                       [np.sin(phi), v*np.cos(phi)]])

        #invert to get the w input - use pseudoinverse to avoid problems
        w = np.linalg.pinv(Aq)@z

        #return w input
        return w

    def eval_input(self, t):
        """
        Solves for the control input to turtlebot i using a CBF-QP controller.
        Inputs:
            t (float): current time in simulation
            i (int): index of turtlebot in the system we wish to control (zero indexed)
        """
        #get the z input to the system
        z = self.eval_z_input(t)

        #SET THE VALUE OF z in the DYNAMICS
        self.observer.dynamics.set_z(z, self.observer.index)

        #get the w input to the system
        w = self.eval_w_input(t, z)

        #integrate the w1 term to get v
        self.vDotInt += w[0, 0]*self.dt

        #return the [v, omega] input
        self._u = np.array([[self.vDotInt, w[1, 0]]]).T
        return self._u
    
    def get_input(self):
        """
        Retrieves input stored in class parameter
        Returns:
            self._u: most recent input stored in class parameter
        """
        return self._u

class TurtlebotCBFQP:
    def __init__(self, observer, barriers, trajectory):
        """
        Class for a CBF-QP controller for a single turtlebot within a larger system. This implementation
        applies a CBF-QP directly over the turtlebot feedback linearizing input, and does not include 
        deadlock resolution steps.
        Args:
            observer (EgoTurtlebotObserver): state observer object for a single turtlebot within the system
            barriers (List of TurtlebotBarrier): List of TurtlebotBarrier objects corresponding to that turtlebot
            traj (Trajectory): trajectory object
        """
        #store input parameters
        self.observer = observer
        self.barriers = barriers
        self.trajectory = trajectory

        #create a nominal controller
        self.nominalController = TurtlebotFBLin(observer, trajectory)

        #store the input parameter (should not be called directly but with get_input)
        self._u = np.zeros((2, 1))

    def eval_input(self, t):
        """
        Solves for the control input to turtlebot i using a CBF-QP controller.
        Inputs:
            t (float): current time in simulation
            i (int): index of turtlebot in the system we wish to control (zero indexed)
            dLock (boolean): include deadlock resolution strategy in controller
        """
        #get the state vector of turtlebot i
        q = self.observer.get_state()

        #get the nominal control input to the system
        self.nominalController.eval_input(t) #call the evaluation function
        kX = self.nominalController.get_input()

        #set up the optimization problem
        opti = ca.Opti()

        #define the decision variable
        u = opti.variable(2, 1)

        #define gamma for CBF tuning
        gamma = 1

        #apply the N-1 barrier constraints
        for bar in self.barriers:
            #get the values of h and hDot
            h, hDot = bar.eval(u, t)

            #compute the optimization constraint
            opti.subject_to(hDot >= -gamma * h)

        cost = (u - kX).T @ (u - kX)

        opti.minimize(cost)
        option = {"verbose": False, "ipopt.print_level": 0, "print_time": 0}
        opti.solver("ipopt", option)

        #solve optimization
        try:
            sol = opti.solve()
            uOpt = sol.value(u) #extract optimal input
            solverFailed = False
        except:
            print("Solver failed!")
            solverFailed = True
            uOpt = np.zeros((2, 1))
        #evaluate and return the input
        self._u = uOpt.reshape((2, 1))
        return self._u
    
    def get_input(self):
        """
        Retrieves input stored in class parameter
        Returns:
            self._u: most recent input stored in class parameter
        """
        return self._u
    
class TurtlebotCBFQPDeadlock(TurtlebotCBFQP):
    def __init__(self, observer, barriers, trajectory):
        """
        Class for CBF QP Controller with embedded deadlock resolution. Apply the CBF directly over z in the 
        feedback linearization step.
        Args:
            observer (EgoTurtlebotObserver): state observer object for a single turtlebot within the system
            barriers (List of TurtlebotBarrierDeadlock): List of TurtlebotBarrier objects corresponding to that turtlebot
            traj (Trajectory): trajectory object
        """
        #call the super initialization function
        super().__init__(observer, barriers, trajectory)

        #store the time step dt for integration
        self.dt = 1/50 #from the control frequency in environment.py

        #store an initial value for the vDot integral
        self.vDotInt = 0

    def eval_z_input(self, t):
        """
        Evaluate the Z input to the system based off of the CBF QP with deadlock resolution.
        Applies a CBF-QP around the nominal z input from feedback linearization.
        """
        #first, get z from the nominal FB lin contorller
        zNom = self.nominalController.eval_z_input(t)

        #now, apply a CBF-QP over this z input using the simplified dynamics. This will be a second order CBF.
        #set up the optimization problem
        opti = ca.Opti()

        #define the decision variable
        u = opti.variable(2, 1)

        #define a weighting variable for CBF-QP to encourage steering
        H = np.diag([1, 0.001])

        #define gamma for CBF tuning (100, 29)
        k1 = 100
        k2 = 29

        #apply the N-1 barrier constraints
        for bar in self.barriers:
            #get the values of h, hDot, hDDot
            h, hDot, hDDot = bar.eval(u, t)

            #compute the optimization constraint
            opti.subject_to(hDDot + k2*hDot + k1*h >= 0)

        #define the cost function
        cost = (u - zNom).T @ H @ (u - zNom)

        opti.minimize(cost)
        option = {"verbose": False, "ipopt.print_level": 0, "print_time": 0}
        opti.solver("ipopt", option)

        #solve optimization
        try:
            sol = opti.solve()
            uOpt = sol.value(u) #extract optimal input
            solverFailed = False
        except:
            print("Solver failed!")
            solverFailed = True
            uOpt = np.zeros((2, 1))
        #evaluate and return the input
        zOpt = uOpt.reshape((2, 1))
        return zOpt
    
    def eval_input(self, t):
        """
        Evaluate the u input to the system using deadlock resolution.
        """
        #first, evaluate the z input based on the CBF constraints
        zOpt = self.eval_z_input(t)

        #SET THE VALUE OF Z IN THE DYNAMICS
        self.observer.dynamics.set_z(zOpt, self.observer.index)

        #next, evaluate w input based on the CBF z input
        wOpt = self.nominalController.eval_w_input(t, zOpt)
        
        #integrate the w1 term to get v
        self.vDotInt += wOpt[0, 0]*self.dt

        #return the [v, omega] input
        self._u = np.array([[self.vDotInt, wOpt[1, 0]]]).T
        return self._u
    
class TurtlebotCBFQPVision(TurtlebotCBFQPDeadlock):
    def __init__(self, observer, barriers, trajectory):
        """
        Class for CBF QP Controller over vision.
        Args:
            observer (EgoTurtlebotObserver): state observer object for a single turtlebot within the system
            barriers (List of TurtlebotBarrierVision): List of TurtlebotBarrierVision objects corresponding to that turtlebot
            traj (Trajectory): trajectory object
            lidar (EgoLidar): lidar object
        """
        #call the super class init
        super().__init__(observer, barriers, trajectory)
    
        

class ControllerManager(Controller):
    def __init__(self, observerManager, barrierManager, trajectoryManager, lidarManager, controlType):
        """
        Class for a CBF-QP controller for a single turtlebot within a larger system.
        Managerial class that points to N controller instances for the system. Interfaces
        directly with the overall system dynamics object.
        Args:
            observer (Observer): state observer object
            controller (String): string of controller type 'TurtlebotCBFQP' or 'TurtlebotFBLin'
            sysTrajectory (SysTrajectory): SysTrajectory object containing the trajectories of all N turtlebots
            nominalController (Turtlebot_FB_Lin): nominal feedback linearizing controller for CBF-QP
        """
        #store input parameters
        self.observerManager = observerManager
        self.barrierManager = barrierManager
        self.trajectoryManager = trajectoryManager
        self.lidarManager = lidarManager
        self.controlType = controlType

        #store the input parameter (should not be called directly but with get_input)
        self._u = None

        #get the number of turtlebots in the system
        self.N = self.observerManager.dynamics.N

        #create a controller dictionary
        self.controllerDict = {}

        #create N separate controllers - one for each turtlebot - use each trajectory in the trajectory dict
        for i in range(self.N):
            #create a controller using the three objects above - add the controller to the dict
            if self.controlType == 'TurtlebotCBFQP':
                #extract the ith trajectory
                trajI = self.trajectoryManager.get_traj_i(i)

                #get the ith observer object
                egoObsvI = self.observerManager.get_observer_i(i)

                #get the ith barrier object
                barrierI = self.barrierManager.get_barrier_list_i(i)

                #create a CBF QP controller
                self.controllerDict[i] = TurtlebotCBFQP(egoObsvI, barrierI, trajI)

            elif self.controlType == 'TurtlebotCBFQPDeadlock':
                #extract the ith trajectory
                trajI = self.trajectoryManager.get_traj_i(i)

                #get the ith observer object
                egoObsvI = self.observerManager.get_observer_i(i)

                #get the ith barrier object
                barrierI = self.barrierManager.get_barrier_list_i(i)

                #create a CBF QP controller
                self.controllerDict[i] = TurtlebotCBFQPDeadlock(egoObsvI, barrierI, trajI)

            elif self.controlType == 'TurtlebotCBFQPVision':
                #vision-based turtlebot CBF-QP

                #extract the ith trajectory
                trajI = self.trajectoryManager.get_traj_i(i)

                #get the ith observer object
                egoObsvI = self.observerManager.get_observer_i(i)

                #get the ith barrier object - assumes that this is a vision-based barrier
                barrierI = self.barrierManager.get_barrier_list_i(i)

                #create a CBF QP controller
                self.controllerDict[i] = TurtlebotCBFQPVision(egoObsvI, barrierI, trajI)

            elif self.controlType == 'TurtlebotFBLin':
                #extract the ith trajectory
                trajI = self.trajectoryManager.get_traj_i(i)

                #get the ith observer object
                egoObsvI = self.observerManager.get_observer_i(i)

                #create a feedback linearizing controller
                self.controllerDict[i] = TurtlebotFBLin(egoObsvI, trajI)

            elif self.controlType == 'Test':
                #define a test type controller - is entirely open loop
                egoObsvI = self.observerManager.get_observer_i(i)
                self.controllerDict[i] = TurtlebotTest(egoObsvI)

            else:
                raise Exception("Invalid Controller Name Error")

    def eval_input(self, t):
        """
        Solve for and return control input for all N turtlebots. Solves for the input ui to 
        each turtlebot in the system and assembles all of the input vectors into a large 
        input vector for the entire system.
        Inputs:
            t (float): time in simulation
        Returns:
            u ((Dynamics.inputDimn x 1)): input vector, as determined by controller
        """
        #initilialize input vector as zero vector - only want to store once all have been updated
        u = np.zeros((self.observerManager.dynamics.inputDimn, 1))

        #loop over the system to find the input to each turtlebot
        for i in range(self.N):
            #solve for the latest input to turtlebot i, store the input in the u vector
            self.controllerDict[i].eval_input(t)
            u[2*i : 2*i + 2] = self.controllerDict[i].get_input()

        #store the u vector in self._u
        self._u = u

        #return the full force vector
        return self._u
    
    def get_input(self):
        """
        Retrieves input stored in class parameter
        Returns:
            self._u: most recent input stored in class paramter
        """
        return self._u