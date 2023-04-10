"""
Master simulation file for a CBF-QP Controller with deadlock resolution.
"""
#Our dependencies
from environment import *
from dynamics import *
from lyapunov_barrier import *
from controller import *
from trajectory import *
from state_estimation import *
from obstacle import *
from utils import *

#system initial condition
N = 5
q0 = gen_init_cond(N)

#create a dynamics object for the double integrator
dynamics = TurtlebotSysDyn(q0, N = N) #set the number of turtlebots to 1 for now

#create an observer manager based on the dynamics object with noise parameters
mean = 0
sd = 0
observerManager = ObserverManager(dynamics, mean, sd)

#set a desired state vector for the system
qD = gen_goal_state(N)

#define a trajectory manager
T = 10
trajManager = TrajectoryManager(q0, qD, T, N)

#define a barrier manager
useDeadLock = True
barrierManager = BarrierManager(N, 3, 2, dynamics, observerManager, 0, dLock = useDeadLock, lidarManager = None)

#Create a controller manager
ctrlType = 'TurtlebotCBFQPDeadlock'
controller = ControllerManager(observerManager, barrierManager, trajManager, None, ctrlType)

#create a simulation environment
env = Environment(dynamics, controller, observerManager, T = T)

#run the simulation
env.run()