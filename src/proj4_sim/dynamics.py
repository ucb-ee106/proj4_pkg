import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation

class Dynamics:
    """
    Skeleton class for system dynamics
    Includes methods for returning state derivatives, plots, and animations
    """
    def __init__(self, x0, stateDimn, inputDimn, relDegree = 1):
        """
        Initialize a dynamics object
        Args:
            x0 (stateDimn x 1 numpy array): initial condition state vector
            stateDimn (int): dimension of state vector
            inputDimn (int): dimension of input vector
            relDegree (int, optional): relative degree of system. Defaults to 1.
        """
        self.stateDimn = stateDimn
        self.inputDimn = inputDimn
        self.relDegree = relDegree
        
        #store the state and input
        self._x = x0
        self._u = None

    def get_input(self):
        """
        Retrieve the input to the system
        """
        return self._u
    
    def get_state(self):
        """
        Retrieve the state vector
        """
        return self._x
        
    def deriv(self, x, u, t):
        """
        Returns the derivative of the state vector
        Args:
            x (stateDimn x 1 numpy array): current state vector at time t
            u (inputDimn x 1 numpy array): current input vector at time t
            t (float): current time with respect to simulation start
        Returns:
            xDot: state_dimn x 1 derivative of the state vector
        """
        return np.zeros((self.state_dimn, 1))
    
    def integrate(self, u, t, dt):
        """
        Integrates system dynamics using Euler integration
        Args:
            u (inputDimn x 1 numpy array): current input vector at time t
            t (float): current time with respect to simulation start
            dt (float): time step for integration
        Returns:
            x (stateDimn x 1 numpy array): state vector after integrating
        """
        #integrate starting at x
        self._x = self.get_state() + self.deriv(self.get_state(), u, t)*dt

        #update the input parameter
        self._u = u

        #return integrated state vector
        return self._x
    
    def get_plots(self, x, u, t):
        """
        Function to show plots specific to this dynamic system.
        Args:
            x ((stateDimn x N) numpy array): history of N states to plot
            u ((inputDimn x N) numpy array): history of N inputs to plot
            t ((1 x N) numpy array): history of N times associated with x and u
        """
        pass
    
    def show_animation(self, x, u, t):
        """
        Function to play animations specific to this dynamic system.
        Args:
            x ((stateDimn x N) numpy array): history of N states to plot
            u ((inputDimn x N) numpy array): history of N inputs to plot
            t ((1 x N) numpy array): history of N times associated with x and u
        """
        pass
    
    
"""
**********************************
PLACE YOUR DYNAMICS FUNCTIONS HERE
**********************************
"""
class TurtlebotSysDyn(Dynamics):
    def __init__(self, x0 , stateDimn = 3, inputDimn = 2, relDegree = 1, N = 3, rTurtlebot = 0.15):
        """
        Init function for a system of N turtlebots.
        Args:
            x0 (NumPy Array): (x1, y1, phi1, ..., xN, yN, phiN) initial condition for all N turtlebots
            stateDimn (Int): state dimension of a SINGLE turtlebot
            inputDimn (Int): input dimension of a SINGLE turtlebot
            relDegree (Int, optional): relative degree of the system
            N (Int, optional): number of turtlebots in the system
            rTurtlebot (float): radius of the turtlebots in the system
        """
        self.N = N #store the number of turtlebots in the system
        self.rTurtlebot = rTurtlebot #store the turtlebot radius
        super().__init__(x0, self.N*stateDimn, self.N*inputDimn, relDegree) #scale the input dimn and rel degree by N for all turtlebots

        #set the initial input self._u to be the zero vector (required for FB linearization)
        self._u = np.zeros((2*self.N, 1))
        self._z = np.zeros((2*self.N, 1)) #store a copy of the augmented input vector as well

    def set_z(self, z, i):
        """
        Function to set the value of z, the augmented input vctor.
        Inputs:
            z ((2N x 1) NumPy Array): Augmented input vector
            i (int): index we wish to place the updated z at
        """
        #store in class attribute
        self._z[2*i : 2*i + 2, 0] = z.reshape((2, ))
    
    def get_z(self):
        """
        Function to return the augmented input vector, z, at any point.
        """
        #retrieve and return augmented input vector
        return self._z

    def deriv(self, x, u, t):
        """
        Returns the derivative of the state vector. Each turtlebot in the system has dynamics of the form:
        [[np.cos(PHI), 0], [np.sin(PHI), 0], [0, 1]]@u -> these dynamics are patterned across the system of N turtlebots.
        Args:
            x (3N x 1 numpy array): current state vector to the whole turtlebot system at time t
            u (2N x 1 numpy array): current input vector to the whole turtlebot system at time t
            t (float): current time with respect to simulation start
        Returns:
            xDot: state_dimn x 1 derivative of the state vector
        """
        xDot = np.zeros((self.stateDimn, 1))
        for i in range(self.N):
            #extract the orientation angle of the Nth turtlebot
            PHI = x[3*i+2, 0] #[x, y, p, x, y, p], [0, 1, 2, 3, 4, 5]
            #find the derivative of the state vector of the Nth turtlebot
            ui = u[2*i: 2*i + 2]
            #calculate xDot - must reshape to slice in
            xDot[3*i:3*i + 3, 0] = (np.array([[np.cos(PHI), 0], [np.sin(PHI), 0], [0, 1]])@ui).reshape((3, ))
        #return the filled in state vector for all N turtlebots
        return xDot

    def show_animation(self, xData, uData, tData, animate = True):
        """
        Shows the animation and visualization of data for this system.
        Args:
            xData (stateDimn x N Numpy array): state vector history array
            u (inputDimn x N numpy array): input vector history array
            t (1 x N numpy array): time history
            animate (bool, optional): Whether to generate animation or not. Defaults to True.
        """
        #Set constant animtion parameters
        FREQ = 50 #control frequency, same as data update frequency
        
        if animate:
            fig, ax = plt.subplots()
            # set the axes limits
            ax.axis([-0.25, 5.25, -0.25, 5.25])
            # set equal aspect such that the circle is not shown as ellipse
            ax.set_aspect("equal")
            # create a set of points in the axes
            points, = ax.plot([],[], marker="o", linestyle='None')
            num_frames = xData.shape[1]-1

            #plot the obstacle
            # circle = plt.Circle((OBS_POS[0], OBS_POS[1]), radius = OBS_R, fc = 'c')
            # plt.gca().add_patch(circle)
            # ax.scatter([GOAL_POS[0]], [GOAL_POS[1]], color = 'y') #goal position
                
            def animate(i):
                x = []
                y = []
                #get the x, y data associated with each turtlebot
                for j in range(self.N):
                    x.append(xData[3*j, i])
                    y.append(xData[3*j+1, i])
                points.set_data(x, y)
                return points,
            
            anim = animation.FuncAnimation(fig, animate, frames=num_frames, interval=1/FREQ*1000, blit=True)
            plt.xlabel("X Position (m)")
            plt.ylabel("Y Position (m)")
            plt.title("Positions of Turtlebots in Space")
            plt.show()

        #Plot the spatial trajectory of the turtlebots
        fig, ax = plt.subplots()
        ax.set_aspect("equal")
        #iterate over each turtlebot state vector
        for j in range(self.N):
            xCoords = xData[3*j, :].tolist() #extract all of the velocity data to plot on the y axis
            yCoords = xData[3*j+1, :].tolist() #remove the last point, get artefacting for some reason
            ax.plot(xCoords[0:-1], yCoords[0:-1])
        # ax.scatter([GOAL_POS[0]], [GOAL_POS[1]], color = 'y') #goal position
        # circle = plt.Circle((OBS_POS[0], OBS_POS[1]), radius = OBS_R, fc = 'c')
        legendList = ["Bot " + str(i) for i in range(self.N)]
        plt.legend(legendList)
        # plt.gca().add_patch(circle)
        plt.xlabel("X Position (m)")
        plt.ylabel("Y Position (m)")
        plt.title("Positions of Turtlebots in Space")
        plt.show()
        
        #Plot each state variable in time
        fig, axs = plt.subplots(5)
        fig.suptitle('Evolution of States and Inputs in Time')
        xlabel = 'Time (s)'
        ylabels = ['X Pos (m)', 'Y Pos (m)', 'Phi (rad)', "V (m/s)", "Omega (rad/s)"]
        indices = [0, 1, 2]
        #plot the states
        for j in range(self.N):
            n = 0 #index in the subplot
            for i in indices:
                axs[n].plot(tData.reshape((tData.shape[1], )).tolist()[0:-1], xData[3*j + i, :].tolist()[0:-1])
                axs[n].set(ylabel=ylabels[n]) #pull labels from the list above
                axs[n].grid()
                n += 1
            #plot the inputs
            for i in range(2):
                axs[i+3].plot(tData.reshape((tData.shape[1], )).tolist()[0:-1], uData[2*j + i, :].tolist()[0:-1])
                axs[i+3].set(ylabel=ylabels[i+3])
                axs[i+3].grid()
        axs[4].set(xlabel = xlabel)
        legendList = ["Bot " + str(i) for i in range(self.N)]
        plt.legend(legendList)
        plt.show()