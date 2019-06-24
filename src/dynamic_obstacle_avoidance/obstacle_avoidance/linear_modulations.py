'''
# Library for the Modulation of Linear Systems

@author Lukas Huber

Copyright (c) 2019 under GPU license. 
'''

import matplotlib.pyplot as plt

import numpy as np
import numpy.linalg as LA

from dynamic_obstacle_avoidance.dynamical_system.dynamical_system_representation import *
from dynamic_obstacle_avoidance.obstacle_avoidance.modulation import *

import warnings

import sys


def obs_avoidance_interpolation_moving(x, xd, obs=[], attractor='none', weightPow=2, repulsive_gammaMargin=0.01):
    # This function modulates the dynamical system at position x and dynamics xd such that it avoids all obstacles obs. It can furthermore be forced to converge to the attractor. 
    # 
    # INPUT
    # x [dim]: position at which the modulation is happening
    # xd [dim]: initial dynamical system at position x
    # obs [list of obstacle_class]: a list of all obstacles and their properties, which present in the local environment
    # attractor [list of [dim]]]: list of positions of all attractors
    # weightPow [int]: hyperparameter which defines the evaluation of the weight
    #
    # OUTPUT
    # xd [dim]: modulated dynamical system at position x
    #

    # print('x', x)
    # print('xd', xd)
    # print('obs', obs)
    
    # Initialize Variables
    N_obs = len(obs) #number of obstacles
    if N_obs ==0:
        return xd
    
    d = x.shape[0]
    Gamma = np.zeros((N_obs))

    if type(attractor)==str:
        if attractor=='default': # Define attractor position
            attractor = np.zeros((d))
            N_attr = 1
        else:
            N_attr = 0            
    else:
        N_attr = 1                 

    # Linear and angular roation of velocity
    xd_dx_obs = np.zeros((d,N_obs))
    xd_w_obs = np.zeros((d,N_obs)) #velocity due to the rotation of the obstacle

    # Modulation matrices
    E = np.zeros((d,d,N_obs))
    D = np.zeros((d,d,N_obs))
    M = np.zeros((d,d,N_obs))
    E_orth = np.zeros((d,d,N_obs))

    # Rotation matrix
    R = np.zeros((d,d,N_obs))


    for n in range(N_obs):
        # Move the position into the obstacle frame of reference
        if obs[n].th_r: # Nonzero value
            R[:,:,n] = compute_R(d,obs[n].th_r)
        else:
            R[:,:,n] = np.eye(d)

        # Move to obstacle centered frame
        x_t = R[:,:,n].T @ (x-obs[n].x0)
        E[:,:,n], D[:,:,n], Gamma[n], E_orth[:,:,n] = compute_modulation_matrix(x_t, obs[n], R[:,:,n])
        
    if N_attr:
        d_a = LA.norm(x - np.array(attractor)) # Distance to attractor
        weight = compute_weights(np.hstack((Gamma, [d_a])), N_obs+N_attr)

    else:
        weight = compute_weights(Gamma, N_obs)
    xd_obs = np.zeros((d))

    
    for n in range(N_obs):
        if d==2:
            xd_w = np.cross(np.hstack(([0,0], obs[n].w)),
                            np.hstack((x-np.array(obs[n].x0),0)))
            xd_w = xd_w[0:2]
        elif d==3:
            xd_w = np.cross( obs[n].w, x-obs[n].x0 )
        else:
            warnings.warn('NOT implemented for d={}'.format(d))

        #the exponential term is very helpful as it help to avoid the crazy rotation of the robot due to the rotation of the object
        exp_weight = np.exp(-1/obs[n].sigma*(np.max([Gamma[n],1])-1))
        xd_obs_n = exp_weight*(np.array(obs[n].xd) + xd_w)

        # xd_obs_n = E_orth[:,:,n].T @ xd_obs_n
        # xd_obs_n[0] = np.max([xd_obs_n[0], 0]) # Onl use orthogonal part 
        # xd_obs_n = E_orth[:,:,n] @ xd_obs_n
        
        xd_obs = xd_obs + xd_obs_n*weight[n]

    xd = xd-xd_obs #computing the relative velocity with respect to the obstacle

    # Create orthogonal matrix
    xd_norm = LA.norm(xd)
    
    if xd_norm: # nonzero
        xd_normalized = xd/xd_norm
    else:
        xd_normalized=xd

    xd_t = np.array([xd_normalized[1], -xd_normalized[0]])

    Rf = np.array([xd_normalized, xd_t]).T

    
    xd_hat = np.zeros((d, N_obs))
    xd_hat_magnitude = np.zeros((N_obs))
    k_ds = np.zeros((d-1, N_obs))
        
    for n in range(N_obs):
        # xd_R = LA.pinv(E[:,:,n]) @ R[:,:,n].T @ xd
        # import pdb; pdb.set_trace() ## DEBUG ##
        
        M[:,:,n] = R[:,:,n] @ E[:,:,n] @ D[:,:,n] @ LA.pinv(E[:,:,n]) @ R[:,:,n].T
        
        xd_hat[:,n] = M[:,:,n] @ xd # velocity modulation
        
        # if False:
        if Gamma[n] < (1+repulsive_gammaMargin): # Safety for implementation (Remove for pure algorithm)
            repulsive_power = 5
            repulsive_factor = 5
            repulsive_gamma = (1+repulsive_gammaMargin)
            
            repulsive_velocity =  ((repulsive_gamma/Gamma[n])**repulsive_power-
                                    repulsive_gamma)*repulsive_factor 
            # print("\n\n Add repulsive vel: {} \n\n".format(repulsive_velocity))
            xd_hat[:,n] += R[:,:,n] @ E[:,0,n] * repulsive_velocity

        xd_hat_magnitude[n] = np.sqrt(np.sum(xd_hat[:,n]**2)) 
        if xd_hat_magnitude[n]: # Nonzero hat_magnitude
            xd_hat_normalized = xd_hat[:,n]/xd_hat_magnitude[n] # normalized direction
        else:
            xd_hat_normalized = xd_hat[:,n]
        
        if not d==2:
            warnings.warn('not implemented for d neq 2')

        xd_hat_normalized_velocityFrame = Rf @ xd_hat_normalized

        # Kappa space - directional space
        k_fn = xd_hat_normalized_velocityFrame[1:]
        kfn_norm = LA.norm(k_fn) # Normalize
        if kfn_norm:# nonzero
            k_fn = k_fn/ kfn_norm
            
        sumHat = np.sum(xd_hat_normalized*xd_normalized)
        if sumHat > 1 or sumHat < -1:
            sumHat = max(min(sumHat, 1), -1)
            warnings.warn('cosinus out of bound!')
            
        # !!!! ??? is this the same as 
        # np.arccos(sumHat) == np.arccos(xd_hat_normalized_velocityFrame[0])
        
        k_ds[:,n] = np.arccos(sumHat)*k_fn.squeeze()

    # xd_hat_magnitude = np.sqrt(np.sum(xd_hat**2, axis=0) ) # TODO - remove as already caclulated
    if N_attr: #nonzero
        k_ds = np.hstack((k_ds, np.zeros((d-1, N_attr)) )) # points at the origin
        xd_hat_magnitude = np.hstack((xd_hat_magnitude, LA.norm((xd))*np.ones(N_attr) ))
        
    # Weighted interpolation for several obstacles
    # weight = weight**weightPow
    # if not LA.norm(weight,2):
        # warnings.warn('trivial weight.')
    # weight = weight/LA.norm(weight,2)
    
    xd_magnitude = np.sum(xd_hat_magnitude*weight)
    k_d = np.sum(k_ds*np.tile(weight, (d-1, 1)), axis=1)

    norm_kd = LA.norm(k_d)
    
    if norm_kd: # Nonzero
        n_xd = Rf.T @ np.hstack((np.cos(norm_kd), np.sin(norm_kd)/norm_kd*k_d ))
    else:
        n_xd = Rf.T @ np.hstack((1, k_d ))

    xd = xd_magnitude*n_xd.squeeze()

    # transforming back from object frame of reference to inertial frame of reference
    xd = xd + xd_obs

    return xd


def compute_modulation_matrix(x_t, obs, R, matrix_singularity_margin=pi/2.0*1.05):
    # The function evaluates the gamma function and all necessary components needed to construct the modulation function, to ensure safe avoidance of the obstacles.
    # Beware that this function is constructed for ellipsoid only, but the algorithm is applicable to star shapes.
    # 
    # Input
    # x_t [dim]: The position of the robot in the obstacle reference frame
    # obs [obstacle class]: Description of the obstacle with parameters
    # R [dim x dim]: Rotation matrix 
    #
    # Output
    # E [dim x dim]: Basis matrix with rows the reference and tangent to the obstacles surface
    # D [dim x dim]: Eigenvalue matrix which is responsible for the modulation
    # Gamma [dim]: Distance function to the obstacle surface (in direction of the reference vector)
    # E_orth [dim x dim]: Orthogonal basis matrix with rows the normal and tangent

    dim = obs.dim
    
    if hasattr(obs, 'rho'):
        rho = np.array(obs.rho)
    else:
        rho = 1

    Gamma = obs.get_gamma(x_t, in_global_frame=False) # function for ellipsoids

    # reference_direction
    # array([0.99979922, 0.02003783])

    # x_t
    # array([-2.33109637,  1.01576377])

    # th_r
    # 0.6981317007977318
    
    normal_vector = obs.get_normal_direction(x_t, in_global_frame=False)
    reference_direction = obs.get_reference_direction(x_t, in_global_frame=False)
    
    

    # Check if there was correct placement of reference point
    Gamma_referencePoint = obs.get_gamma(obs.reference_point)
    if Gamma_referencePoint >= 1:
                
        # surface_position = obs.get_obstace_radius* x_t/LA.norm(x_t)
        # direction_surface2reference = obs.get_reference_point()-surface_position

        # Per default negative
        referenceNormal_angle = np.arccos(reference_direction.T @ normal_vector)
        
        # if referenceNormal_angle < (matrix_singularity_margin):
            # x_global = obs.transform_relative2global(x_t)
            # plt.quiver(x_global[0],x_global[1], normal_vector[0], normal_vector[1], 'r')
            # plt.quiver(x_global[0],x_global[1], normal_vector[0], normal_vector[1], color='r')
            # plt.quiver(x_global[0],x_global[1], reference_direction[0], reference_direction[1], color='b')
            
            # referenceNormal_angle = np.min([0, referenceNormal_angle-pi/2.0])
    
            # Gamma_convexHull = 1*referenceNormal_angle/(matrix_singularity_margin-pi/2.0)
            # Gamma = np.max([Gamma, Gamma_convexHull])
            
            # reference_orthogonal = (normal_vector -
                                    # reference_direction * reference_direction.T @ normal_vector)
            # normal_vector = (reference_direction*np.sin(matrix_singularity_margin)
                             # + reference_orthogonal*np.cos(matrix_singularity_margin))

            # plt.quiver(x_global[0],x_global[1], normal_vector[0], normal_vector[1], color='g')
            # plt.ion()
            
    E_orth = np.zeros((dim, dim))
    
    # Create orthogonal basis matrix        
    E_orth[:,0] = normal_vector# Basis matrix

    for ii in range(1,dim):

        if dim ==2:
            E_orth[0, 1] = E_orth[1, 0]
            E_orth[1, 1] = - E_orth[0, 0]
            
        # TODO higher dimensions
        # E[:dim-(ii), ii] = normal_vector[:dim-(ii)]*normal_vector[dim-(ii)]
        # E[dim-(ii), ii] = -np.dot(normal_vector[:dim-(ii)], normal_vector[:dim-(ii)])
        # E_orth[:, ii] = E_orth[:, ii]/LA.norm(E_orth[:, ii])

    E = np.copy((E_orth))
    E[:,0] = -reference_direction

    eigenvalue_reference, eigenvalue_tangent = calculate_eigenvalues(Gamma)
    D = np.diag(np.hstack((eigenvalue_reference, np.ones(dim-1)*eigenvalue_tangent)))

    # print('norm ref prod', reference_direction.T@normal_vector)
    # import pdb; pdb.set_trace() ## DEBUG ##
    
    return E, D, Gamma, E_orth


def calculate_eigenvalues(Gamma, rho=1):
    if Gamma<=1:# point inside the obstacle
        delta_eigenvalue = 1 
    else:
        delta_eigenvalue = 1./abs(Gamma)**(1/rho)
    
    eigenvalue_reference = 1 - delta_eigenvalue
    eigenvalue_tangent = 1 + delta_eigenvalue
    return eigenvalue_reference, eigenvalue_tangent


def obs_avoidance_rk4(dt, x, obs, obs_avoidance=obs_avoidance_interpolation_moving, ds=linearAttractor, x0=False):
    # Fourth order integration of obstacle avoidance differential equation
    # NOTE: The movement of the obstacle is considered as small, hence position and movement changed are not considered. This will be fixed in future iterations.

    if type(x0)==bool:
        x0 = np.zeros(np.array(x).shape[0])

    # k1
    xd = ds(x, x0)
    xd = velConst_attr(x, xd, x0)
    xd = obs_avoidance(x, xd, obs)
    k1 = dt*xd

    # k2
    xd = ds(x+0.5*k1, x0)
    xd = velConst_attr(x, xd, x0)
    xd = obs_avoidance(x+0.5*k1, xd, obs)
    k2 = dt*xd

    # k3
    xd = ds(x+0.5*k2, x0)
    xd = velConst_attr(x, xd, x0)
    xd = obs_avoidance(x+0.5*k2, xd, obs)
    
    k3 = dt*xd

    # k4
    xd = ds(x+k3, x0)
    xd = velConst_attr(x, xd, x0)
    xd = obs_avoidance(x+k3, xd, obs)
    k4 = dt*xd

    # x final
    x = x + 1./6*(k1+2*k2+2*k3+k4) # + O(dt^5)

    return x
