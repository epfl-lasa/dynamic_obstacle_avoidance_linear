'''
@author lukashuber
@date 2019-02-03
'''

import numpy as np
from numpy import linalg as LA

from math import ceil, sin, cos, sqrt

import matplotlib.pyplot as plt # for debugging

import warnings

class Intersection_matrix():
    # Matrix uses less space this way this is useful with many obstacles! e.g. dense crowds
    # Symmetric matrix with zero as diagonal values
    def __init__(self, n_obs, dim=2):
        self._intersection_list = [False for ii in range(int((n_obs-1)*n_obs/2))]
        self._dim = n_obs-1

    def set(self, row, col, value):
        ind = self.get_index(row,col)
        self._intersection_list[ind] = value

    def get(self, row, col):
        ind = self.get_index(row,col)
        return self._intersection_list[ind]

    def get_intersection_matrix(self):
        # Maybe not necessary function
        space_dim = 2
        matr = np.zeros((space_dim, self._dim+1, self._dim+1))
        for col in range(self._dim+1):
            for row in range(self._dim+1):
                if col!=row and type(self.get(row,col))!= bool:
                    matr[:,col,row] = self.get(row,col)
        return matr

    def get_bool_triangle_matrix(self):
        intersection_exists_matrix = np.zeros((self._dim+1,self._dim+1), dtype=bool)

        for col in range(self._dim+1):
            for row in range(col+1, self._dim+1):
                if type(self.get(row,col))== bool and (self.get(row,col) == False):
                    continue
                intersection_exists_matrix[row, col] = True

        return intersection_exists_matrix

    def get_bool_matrix(self):
        boolMat = self.get_bool_triangle_matrix()
        return boolMat + boolMat.T

    def get_index(self, row, col):
        if row > np.abs(self._dim):
            print('WARNING: Fist object index out of bound.')
            row = 0
        if row < 0: row = self._dim+1-row
            
        if col > np.abs(self._dim):
            print('WARNING: Second object index out of bound.')
            col = 1
        if col < 0: row = self._dim+1-col
        
        if row == col:
            print('WARNING: Self collision observation meainingless.')
            row, col = 1, 0

        if col > row: # inverse indices
            col, row = row, col

        return int((row-col-1) + col*((self._dim) + self._dim-(col-1) )*0.5)


def obs_common_section(obs):
    #OBS_COMMON_SECTION finds common section of two ore more obstacles 
    # at the moment only solution in two d is implemented

    N_obs = len(obs)
    # No intersection region 
    if not N_obs:
        return []

    # Intersction surface
    intersection_obs = []
    intersection_sf = []
    intersection_sf_temp = []
    it_intersect = -1

    # Ext for more dimensions
    d = len(obs[0].x0)

    N_points = 30 # Choose number of points each iteration
    Gamma_steps = 5 # Increases computational cost
    
    rotMat = np.zeros((d,d, N_obs))
    
    for it_obs in range(N_obs):
        rotMat[:,:,it_obs] = np.array(( obs[it_obs].rotMatrix ))
        obs[it_obs].draw_ellipsoid()
        obs[it_obs].cent_dyn = np.copy(obs[it_obs].x0) # set default value
        
    for it_obs1 in range(N_obs):
        intersection_with_obs1 = False
        # Check if current obstacle 'it_obs1' has already an intersection with another
        # obstacle 
        memberFound = False
        for ii in range(len(intersection_obs)):
            if it_obs1 in intersection_obs[ii]:
                memberFound=True
                continue

        for it_obs2 in range(it_obs1+1,N_obs):
            # Check if obstacle has already an intersection with another
            # obstacle 
            memberFound=False
            for ii in range(len(intersection_obs)):
                if it_obs2 in intersection_obs[ii]:
                    memberFound=True 
                    continue

            if memberFound: continue 

            if intersection_with_obs1:# Modify intersecition part
                obsCloseBy = False

                # Roughly check dimensions before starting expensive calculation
                #for ii in intersection_obs:
                    # if it_intersect[ii]:
                    #     if LA.norm(obs[ii].x0 - obs[it_obs2].x0) < np.lina(obs[ii].a)*obs[ii].sf + np.linalg.norm(obs[it_obs2].a)*obs[it_obs2].sf:
                    #         # Obstacles to far apart
                    #         obsCloseBy = True
                    #         break

                #if obsCloseBy:
                if True:
                    N_inter = intersection_sf[it_intersect].shape[1] # Number of intersection points

                    ## R = compute_R(d,obs[it_obs2].th_r)
                    Gamma_temp = ( rotMat[:,:,it_obs2].T @(intersection_sf[it_intersect]-np.tile(obs[it_obs2].x0,(N_inter,1)).T )/ np.tile(obs[it_obs2].a,(N_inter,1)).T ) ** (2*np.tile(obs[it_obs2].p,(N_inter,1)).T)
                    Gamma = np.sum( 1/obs[it_obs2].sf *Gamma_temp, axis=0 )

                    ind = Gamma<1
                    
                    if sum(ind):
                        intersection_sf[it_intersect] = intersection_sf[it_intersect][:,ind]
                        intersection_obs[it_intersect] = intersection_obs[it_intersect] + [it_obs2]
            else:
                # Roughly check dimensions before starting expensive calculation
                #if sqrt(sum((np.array(obs[it_obs1].x0)-np.array(obs[it_obs2].x0))**2)) < max(obs[it_obs1].a)*obs[it_obs1].sf + max(obs[it_obs2].a)*obs[it_obs2].sf: # Obstacles are close enough
                if True:
                    
                    # get all points of obs2 in obs1
                    # R = compute_R(d,obs[it_obs1].th_r)
                    # \Gamma = \sum_[i=1]^d (xt_i/a_i)^(2p_i) == 1
                    N_points = len(obs[it_obs1].x_obs_sf)
                    
                    Gamma_temp = (rotMat[:,:,it_obs1].T @  (np.array(obs[it_obs2].x_obs_sf).T-np.tile(obs[it_obs1].x0,(N_points,1)).T ) / np.tile(obs[it_obs1].a, (N_points,1)).T )
                    Gamma = np.sum( (1/obs[it_obs1].sf *  Gamma_temp) ** (2*np.tile(obs[it_obs1].p, (N_points,1)).T), axis=0)
                    intersection_sf_temp = np.array(obs[it_obs2].x_obs_sf)[Gamma<1,:].T

                    # Get all poinst of obs1 in obs2
                    #                 R = compute_R(d,obs[it_obs2].th_r)
                    Gamma_temp = ( rotMat[:,:,it_obs2].T @ (np.array(obs[it_obs1].x_obs_sf).T-np.tile(obs[it_obs2].x0,(N_points,1)).T ) / np.tile(obs[it_obs2].a, (N_points,1)).T )
                    Gamma = np.sum(( 1/obs[it_obs2].sf *  Gamma_temp)  ** (2*np.tile(obs[it_obs2].p, (N_points,1)).T), axis=0 )
                    intersection_sf_temp = np.hstack((intersection_sf_temp, np.array(obs[it_obs1].x_obs_sf)[Gamma<1,:].T ) )

                    if intersection_sf_temp.shape[1] > 0:
                        it_intersect = it_intersect + 1
                        intersection_with_obs1 = True
                        intersection_sf.append(intersection_sf_temp)
                        intersection_obs.append([it_obs1,it_obs2])

                        # Increase resolution by sampling points within
                        # obstacle, too
                                                    # obstaacles of 2 in 1
                        for kk in range(2):
                            
                            if kk == 0:
                                it_obs1_ = it_obs1
                                it_obs2_ = it_obs2

                            elif kk ==1: # Do it both ways
                                it_obs1_ = it_obs2
                                it_obs2_ = it_obs1

                            for ii in range(1,Gamma_steps):
                                N_points_interior = ceil(N_points/Gamma_steps*ii)
                                
                                #print('a_temp_outside', np.array(obs[it_obs1_].a)/Gamma_steps*ii)
                                x_obs_sf_interior= obs[it_obs1_].draw_ellipsoid(numPoints=N_points_interior, a_temp = np.array(obs[it_obs1_].a)/Gamma_steps*ii)

                                resolution = x_obs_sf_interior.shape[1] # number of points 

                                # Get Gamma value
                                Gamma = np.sum( (1/obs[it_obs2_].sf *  rotMat[:,:,it_obs2_].T @ (x_obs_sf_interior-np.tile(obs[it_obs2_].x0,(resolution,1)).T ) / np.tile(obs[it_obs2_].a, (resolution,1)).T ) ** (2*np.tile(obs[it_obs2_].p, (resolution,1)).T), axis=0)
                                intersection_sf[it_intersect] = np.hstack((intersection_sf[it_intersect],x_obs_sf_interior[:,Gamma<1] ))
                                
                            # Check center point
                            if 1 > sum( (1/obs[it_obs2_].sf*rotMat[:,:,it_obs2_].T @ ( np.array(obs[it_obs1_].x0) - np.array(obs[it_obs2_].x0) )/ np.array(obs[it_obs2_].a) ) ** (2*np.array(obs[it_obs2_].p))):
                                intersection_sf[it_intersect] = np.hstack([intersection_sf[it_intersect],np.tile(obs[it_obs1_].x0,(1,1)).T ] )


    #if intersection_with_obs1 continue 
    if len(intersection_sf)==0:
        return  []

    #plt.plot(intersection_sf[0][0,:], intersection_sf[0][1,:], 'r.')
    
    for ii in range(len(intersection_obs)):
    #     plot(intersection_sf[ii](1,:),intersection_sf[ii](2,:),'x')
        intersection_sf[ii] = np.unique(intersection_sf[ii], axis=1)

        # Get numerical mean
        x_center_dyn= np.mean(intersection_sf[ii], axis=1)
        #plt.plot(x_center_dyn[0], x_center_dyn[1], 'go')
        
        for it_obs in intersection_obs[ii]:
            obs[it_obs].center_dyn = x_center_dyn

        # sort points according to angle
    #     intersec_sf_cent = intersection_sf - repmat(x_center_dyn,1,size(intersection_sf,2))


        # TODO - replace atan2 for speed
    #     [~, ind] = sort( atan2(intersec_sf_cent(2,:), intersec_sf_cent(1,:)))

    #     intersection_sf = intersection_sf(:, ind)
    #     intersection_sf = [intersection_sf, intersection_sf(:,1)]

    #     intersection_obs = [1:size(obs,2)]

    return intersection_obs 

