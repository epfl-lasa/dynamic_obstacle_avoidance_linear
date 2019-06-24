# coding: utf-8
import numpy as np
from math import sin, cos, pi, ceil
import warnings, sys

import numpy as np
import numpy.linalg as LA

from dynamic_obstacle_avoidance.obstacle_avoidance.modulation import *

import matplotlib.pyplot as plt

from obstacle import *

from autograd import grad

def RegressionObstacle(Obstacle):
    def __init__(self):
        pass
    
    def set_center_pos(self, center_position, reset_reference=True):
        self.center_position = center_position
        self.reference_point = np.zeros(self.dim)

        
    def learn_surface(self, val_input, val_regress,
                      polar_surface_representation=False,
                      regression_type="svr"):
        self.polar_surface_representation = polar_surface_representation

        if self.polar_surface_representation:
            pass

        if regression_type=="svr":
            C = 10
            epsilon = 0.1
            self.surface_regression  = sklearn.svm.SVR(kernel=’rbf’,
                                                       C=C, epsilon=epsilon )
            
            self.surface_regression.fit(val_input, val_regress)
            self.grad_surface = grad(self.surface_regression.predict)
        pass

    
    def transform_cartesian2polar(self, points, center_point=None):
        # only 2D
        if type(center_point)==None:
            center_point = self.center_point

        points = point - np.tile(center_point, (points.shape[1],1)).T
        magnitude = LA.norm(points, axis=1)
        angle = np.arctan2(points[1,:], points[0,:])
        
        # output: [r, phi]
        return magnitude, angle

    
    def transform_polar2cartesian(self, magnitude, angle, center_point=None):
        # points = [r, phi]
        if type(center_point)==None:
            center_point = self.center_point 
        points = (magnitude * np.vstack((np.cos(angle), np.sin(angle)))
                  + np.tile(center_point, (radius.shape[0],1)).T )
        return points

    
    def get_normal_direction(self, position, delta_x=0.1):
        # only 2D
        if self.polar_surface_representation:
            

            self.grad_surface(position)
        else:
            return self.grad_surface(position)
        
    
    def get_gamma(self, position, in_global_frame=False, gamma_scaling=1.0):
        if in_global_frame:
            position = self.transform_global2relative(position)
            
        dist_origin2surf = self.surface_regression.predict(position)

        if self.polar_surface_representation:
            dist_origin2position = LA.norm(position)
        else:
            dist_origin2position = position[1]

        return 1+(dist_origin2position-dist_origin2surf)/gamma_scaling


def GaussianEllipseObstacle(Obstacle):
    
    pass

