import numpy as np
#File containing classes involved in managing obstacle avoidance.

class Circle:
    def __init__(self, r, c):
        """
        Init function for a planar circular obstacle.
        Represents the geometry of the obstacle.
        Args:
            r (float): radius of circular obstacle
            c (2x1 numpy array): center position of obstacle
        """
        self._r = r
        self._c = c
        
    def get_radius(self):
        """
        Return the radius of the obstacle
        """
        return self._r
    
    def get_center(self):
        """
        Return the center of the obstacle in the world frame.
        Note that z coordinate should be 0
        """
        return self._c