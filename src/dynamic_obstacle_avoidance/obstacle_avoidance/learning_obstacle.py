import numpy as np
from math import sin, cos, pi, ceil
import warnings, sys

import numpy as np
import numpy.linalg as LA

from dynamic_obstacle_avoidance.obstacle_avoidance.modulation import *

import matplotlib.pyplot as plt

from obstacle import *

def RegressionObstacle(Obstacle):
    def __init__(self):
        pass

    def set_center_pos(self, center_position, reset_reference=True):
        self.center_position = center_position
        self.reference_point = np.zeros(self.dim)

    def learn_surface(self, polar_surface_representation=False):
        self.polar_surface_representation = polar_surface_representation
        
        pass

    def transform_cartesian2polar(self, points, center_point=None):
        # only 2D
        if type(center_point)==None:
            center_point = self.center_point

        points = point - np.tile(center_point, (points.shape[1],1)).T
        magnitude = LA.norm(points, axis=1)
        angle = np.arctan2(points[1,:], points[0,:])
        
        # output: [r, phi]
        return r, phi

    def transform_polar2caresian(self, radius, angle, center_point=None):
        # points = [r, phi]
        if type(center_point)==None:
            center_point = self.center_point 
        points = (radius * np.vstack((np.cos(angle), np.sin(angle)))
                  + np.tile(center_point, (radius.shape[0],1)).T )
        return points

    def get_normal_direction(self):
        pass

    def get_gamma(self):
        pass

    
def GaussianEllipseObstacle(Obstacle):
    pass
                                 
