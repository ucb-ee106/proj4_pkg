"""
Master simulation file for a feedback linearizing controller (pure feedback linearization)
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
N = 1 #just initialize with a single robot here (no safety required)
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

#Create a controller manager
controller = ControllerManager(observerManager, None, trajManager, None, 'TurtlebotFBLin')

#create a simulation environment
env = Environment(dynamics, controller, observerManager, T = T)

#run the simulation
env.run()