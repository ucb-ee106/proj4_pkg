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
        #get the state of the turtlebot
        qe = self.observer.get_state()

        #get the derivative of q of the turtlebot
        qeDot = self.observer.get_vel()

        #get the trajectory states
        xD, vD, aD = self.trajectory.get_state(t)

        """
        TODO: Your code here
        In this function, you will determine the augmented input z = [z1, z2]^T to the turtlebot using a feedback
        linearizing controller. Several useful terms for this controller have been extracted above for you.

        They are defined as follows:
            qe (3x1 NumPy Array): current state vector [x, y, phi]^T of the turtlebot we wish to control
            qeDot (3x1 NumPy Array): current derivative of the state vector [xDot, yDot, phiDot]^T of the turtlebot we wish to control
            xD, vD, aD (3x1 NumPy Arrays): desired state vector, desired derivative of state vector, desired second derivative of state vector
            from a desired trajectory. These give the necessary parameters of the trajectory we wish the turtlebot to track. Recall that we 
            only care about the [x, y] components of these trajectories, as our outputs are x and y.

        NOTE: In this function, the subscript 'e' stands for 'ego' - this is the name of the turtlebot we wish to control.
        """

        #TODO: calculate the z input to the system

        z = ...

        #return the z input
        return z
    
    def eval_w_input(self, t, z):
        """
        Solve for the w input to the system
        Inputs:
            t (float): current time in the system
            z ((2x1) NumPy Array): z input to the system
        """
        #get the state of the turtlebot
        qe = self.observer.get_state()

        #get the derivative of q of the turtlebot
        qeDot = self.observer.get_vel()

        #get the trajectory states
        xD, vD, aD = self.trajectory.get_state(t)

        """
        TODO: Your code here
        In this function, you will determine the augmented input w = [w1, w2]^T to the turtlebot using a feedback
        linearizing controller. Several useful terms for this controller have been extracted above for you.

        They are defined as follows:
            z (2x1 NumPy Array): z input to the system as determined by the function eval_z_input() above
            qe (3x1 NumPy Array): current state vector [x, y, phi]^T of the turtlebot we wish to control
            qeDot (3x1 NumPy Array): current derivative of the state vector [xDot, yDot, phiDot]^T of the turtlebot we wish to control
            xD, vD, aD (3x1 NumPy Arrays): desired state vector, desired derivative of state vector, desired second derivative of state vector
            from a desired trajectory. These give the necessary parameters of the trajectory we wish the turtlebot to track. Recall that we 
            only care about the [x, y] components of these trajectories, as our outputs are x and y.

        NOTE: In this function, the subscript 'e' stands for 'ego' - this is the name of the turtlebot we wish to control.
        """

        #TODO: calculate the w input to the system
        w = ...

        return w

    def eval_input(self, t):
        """
        Solves for the control input to the ego turtlebot using a feedback linearizing controller.
        Inputs:
            t (float): current time in simulation
        """

        #get the z input to the system
        z = self.eval_z_input(t)

        #SET THE VALUE OF z in the DYNAMICS - DO NOT CHANGE THIS LINE
        self.observer.dynamics.set_z(z, self.observer.index)

        #get the w input to the system
        w = self.eval_w_input(t, z)

        """
        TODO: Your code here
        In this function, you will determine the input u = [v, omega]^T to the turtlebot using a feedback
        linearizing controller. Several useful terms for this controller have been extracted above for you.

        Once you've calculated your control input, you should store it in the variable self._u at the bottom of this function.
        Hints: To keep track of the integrated input, define an integral parameter in the class that you can refer to each time
        the function is called.
        
        NOTE: The time step for integration is stored in self.dt if you choose to use a numerical integral in your code.
        """

        #TODO: calculate u = [v, omega]^T using the augmented z and w inputs (which have been called above already)

        self._u = ...

        #return the [v, omega] input
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
        Class for a CBF-QP controller for a single turtlebot within a larger system.
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
        """
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
        #first, get zNominal from the nominal FB lin contorller
        zNom = self.nominalController.eval_z_input(t)

        """
        TODO: Your code here
        In this function, you will determine the SAFE input z to the turtlebot using a CBF-QP.
        Several useful terms for this controller have been extracted above for you.

        They are defined as follows:
            zNom ((2x1) NumPy Array): this is the nominal tracking input to the system, generated by the feedback linearizing controller above
            self.barriers (List): this is a list of TurtlebotBarrierDeadlock objects, corresponding to all of the barrier functions
            associated with this particular turtlebot. 
            
        Recall that to get the value of a barrier function and its derivatives, we can use the following syntax 
        (defined in lyapunov_barrier.py), where bar is an object within the self.barriers list:
            h, hDot, hDDot = bar.eval(u, t)
        Where u is a 2x1 input vector.
        """

        #TODO: use a CBF-QP with Casadi to calculate the safe z input to the system

        zSafe = ...

        #return zSafe
        return zSafe

    def eval_w_input(self, t, z):
        """
        Solve for the w input to the system
        Inputs:
            t (float): current time in the system
            z ((2x1) NumPy Array): z input to the system
        """
        #get the state of the turtlebot
        qe = self.observer.get_state()

        #get the derivative of q of the turtlebot
        qeDot = self.observer.get_vel()

        #get the trajectory states
        xD, vD, aD = self.trajectory.get_state(t)

        """
        TODO: Your code here
        In this function, you will determine the augmented input w = [w1, w2]^T to the turtlebot using a feedback
        linearizing controller. Several useful terms for this controller have been extracted above for you.

        They are defined as follows:
            z (2x1 NumPy Array): z input to the system as determined by the function eval_z_input() above
            qe (3x1 NumPy Array): current state vector [x, y, phi]^T of the turtlebot we wish to control
            qeDot (3x1 NumPy Array): current derivative of the state vector [xDot, yDot, phiDot]^T of the turtlebot we wish to control
            xD, vD, aD (3x1 NumPy Arrays): desired state vector, desired derivative of state vector, desired second derivative of state vector
            from a desired trajectory. These give the necessary parameters of the trajectory we wish the turtlebot to track. Recall that we 
            only care about the [x, y] components of these trajectories, as our outputs are x and y.

        Hints: Should this function be different than for the pure FB linearization case?
        NOTE: In this function, the subscript 'e' stands for 'ego' - this is the name of the turtlebot we wish to control.
        """

        #TODO: calculate the w input to the system based on the z input passed into the function
        w = ...

        return w
    
    def eval_input(self, t):
        """
        Evaluate the u input to the system using deadlock resolution.
        """
        #first, evaluate the optimal z input based on the CBF constraints
        zOpt = self.eval_z_input(t)

        #SET THE VALUE OF Z IN THE DYNAMICS - DO NOT CHANGE THIS LINE
        self.observer.dynamics.set_z(zOpt, self.observer.index)

        #next, evaluate w input based on the CBF z input
        wOpt = self.eval_w_input(t, zOpt)

        """
        TODO: Your code here
        In this function, you will determine the input u = [v, omega]^T to the turtlebot using a feedback
        linearizing controller. Several useful terms for this controller have been extracted above for you.

        Once you've calculated your control input, you should store it in the variable self._u at the bottom of this function.
        Hints: To keep track of the integrated input, define an integral parameter in the class that you can refer to each time
        the function is called.
        
        NOTE: The time step for integration is stored in self.dt if you choose to use a numerical integral in your code.
        """
        
        #TODO: calculate self._u = [v, omega]^T

        self._u = ...

        #return self._u
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

    def eval_z_input(self, t):
        """
        Evaluate the Z input to the system based off of the CBF QP with deadlock resolution.
        Applies a CBF-QP around the nominal z input from feedback linearization.
        """
        #first, get zNominal from the nominal FB lin contorller
        zNom = self.nominalController.eval_z_input(t)

        """
        TODO: Your code here
        In this function, you will determine the SAFE input z to the turtlebot using a CBF-QP.
        Several useful terms for this controller have been extracted above for you.

        They are defined as follows:
            zNom ((2x1) NumPy Array): this is the nominal tracking input to the system, generated by the feedback linearizing controller above
            self.barriers (List): this is a list of TurtlebotBarrierVision objects, corresponding to all of the barrier functions
            associated with this particular turtlebot. These are the PURELY VISION-BASED BARRIER FUNCTIONS.
            
        Recall that to get the value of a barrier function and its derivatives, we can use the following syntax 
        (defined in lyapunov_barrier.py), where bar is an object within the self.barriers list:
            h, hDot, hDDot = bar.eval(u, t)
        Where u is a 2x1 input vector.
        """

        #TODO: use a CBF-QP with Casadi to calculate the safe z input to the system

        zSafe = ...

        #return zSafe
        return zSafe

    def eval_w_input(self, t, z):
        """
        Solve for the w input to the system
        Inputs:
            t (float): current time in the system
            z ((2x1) NumPy Array): z input to the system
        """
        #get the state of the turtlebot
        qe = self.observer.get_state()

        #get the derivative of q of the turtlebot
        qeDot = self.observer.get_vel()

        #get the trajectory states
        xD, vD, aD = self.trajectory.get_state(t)

        """
        TODO: Your code here
        In this function, you will determine the augmented input w = [w1, w2]^T to the turtlebot using a feedback
        linearizing controller. Several useful terms for this controller have been extracted above for you.

        They are defined as follows:
            z (2x1 NumPy Array): z input to the system as determined by the function eval_z_input() above
            qe (3x1 NumPy Array): current state vector [x, y, phi]^T of the turtlebot we wish to control
            qeDot (3x1 NumPy Array): current derivative of the state vector [xDot, yDot, phiDot]^T of the turtlebot we wish to control
            xD, vD, aD (3x1 NumPy Arrays): desired state vector, desired derivative of state vector, desired second derivative of state vector
            from a desired trajectory. These give the necessary parameters of the trajectory we wish the turtlebot to track. Recall that we 
            only care about the [x, y] components of these trajectories, as our outputs are x and y.

        Hints: Should this function be different than for the pure FB linearization case?
        NOTE: In this function, the subscript 'e' stands for 'ego' - this is the name of the turtlebot we wish to control.
        """

        #TODO: calculate the w input to the system based on the z input passed into the function
        w = ...

        return w
    
    def eval_input(self, t):
        """
        Evaluate the u input to the system using deadlock resolution.
        """
        #first, evaluate the optimal z input based on the CBF constraints
        zOpt = self.eval_z_input(t)

        #SET THE VALUE OF Z IN THE DYNAMICS - DO NOT CHANGE THIS LINE
        self.observer.dynamics.set_z(zOpt, self.observer.index)

        #next, evaluate w input based on the CBF z input
        wOpt = self.eval_w_input(t, zOpt)

        """
        TODO: Your code here
        In this function, you will determine the input u = [v, omega]^T to the turtlebot using a feedback
        linearizing controller. Several useful terms for this controller have been extracted above for you.

        Once you've calculated your control input, you should store it in the variable self._u at the bottom of this function.
        Hints: To keep track of the integrated input, define an integral parameter in the class that you can refer to each time
        the function is called.
        
        NOTE: The time step for integration is stored in self.dt if you choose to use a numerical integral in your code.
        """
        
        #TODO: calculate self._u = [v, omega]^T

        self._u = ...

        #return self._u
        return self._u
    
        

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