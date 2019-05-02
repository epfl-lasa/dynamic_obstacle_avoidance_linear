'''
Obstacle Avoidance Algorithm script with vecotr field

@author LukasHuber
@date 2018-02-15
'''

# General classes
import numpy as np
from numpy import pi
import copy
import time

from dynamic_obstacle_avoidance.dynamical_system import *
from dynamic_obstacle_avoidance.obstacle_avoidance.linear_modulations import *
from dynamic_obstacle_avoidance.obstacle_avoidance.obs_common_section import *
from dynamic_obstacle_avoidance.obstacle_avoidance.obs_dynamic_center_3d import *
# from dynamic_obstacle_avoidance.obstacle_avoidance.obstacle import Obstacle
# from dynamic_obstacle_avoidance.visualization.vector_field_visualization import Simulation_vectorFields
# from dynamic_obstacle_avoidance.visualization.animated_simulation import *
# from dynamic_obstacle_avoidance.visualization.animated_simulation_ipython import *

def pltLines(pos0, pos1, xlim=[-100,100], ylim=[-100,100]):
    if pos1[0]-pos0[0]: # m < infty
        m = (pos1[1] - pos0[1])/(pos1[0]-pos0[0])
        
        ylim=[0,0]
        ylim[0] = pos0[1] + m*(xlim[0]-pos0[0])
        ylim[1] = pos0[1] + m*(xlim[1]-pos0[0])
    else:
        xlim = [pos1[0], pos1[0]]
    
    plt.plot(xlim, ylim, '--', color=[0.3,0.3,0.3], linewidth=2)


def plot_streamlines(points_init, ax, obs=[], attractorPos=[0,0],
                     dim=2, dt=0.05, max_simu_step=1000, convergence_margin=0.03):
    n_points = np.array(points_init).shape[1]

    x_pos = np.zeros((dim, max_simu_step+1, n_points))
    x_pos[:,0,:] = points_init

    for iSim in range(max_simu_step):
        for j in range(n_points):
            x_pos[:, iSim+1,j] = obs_avoidance_rk4(
                dt, x_pos[:,iSim, j], obs, x0=attractorPos,
                obs_avoidance=obs_avoidance_interpolation_moving)

         # Check convergence
        if (np.sum((x_pos[:, iSim+1, :]-np.tile(attractorPos, (n_points,1)).T)**2)
            < convergence_margin):
            x_pos = x_pos[:, :iSim+2, :]
            break

    for j in range(n_points):
        ax.plot(x_pos[0, :, j], x_pos[1, :, j], '--', lineWidth=4)
        ax.plot(x_pos[0, 0, j], x_pos[1, 0, j], 'k*', markeredgewidth=4, markersize=13)
        

    # return x_pos

def Simulation_vectorFields(x_range=[0,10],y_range=[0,10], point_grid=10, obs=[], sysDyn_init=False, xAttractor = np.array(([0,0])), saveFigure = False, figName='default', noTicks=True, showLabel=True, figureSize=(16.,13), obs_avoidance_func=obs_avoidance_interpolation_moving, attractingRegion=False, drawVelArrow=False, colorCode=False, streamColor=[0.05,0.05,0.7], obstacleColor=[], plotObstacle=True, plotStream=True, figHandle=[], alphaVal=1, dynamicalSystem=linearAttractor, draw_vectorField=True, points_init=[], show_obstacle_number=False):

    dim = 2

    # Numerical hull of ellipsoid 
    for n in range(len(obs)): 
        obs[n].draw_ellipsoid(numPoints=50) # 50 points resolution 

    # Adjust dynamic center
    intersection_obs = obs_common_section(obs)
    if len(obs)>1:
        dynamic_center_3d(obs, intersection_obs)

    if len(figHandle): 
        fig_ifd, ax_ifd = figHandle[0], figHandle[1] 
    else:
        fig_ifd, ax_ifd = plt.subplots(figsize=figureSize) 


    if plotObstacle:
        obs_polygon = []
        obs_polygon_sf = []

        for n in range(len(obs)):
            x_obs_sf = obs[n].x_obs_sf # todo include in obs_draw_ellipsoid
            
            plt.plot([x_obs_sf[i][0] for i in range(len(x_obs_sf))],
                [x_obs_sf[i][1] for i in range(len(x_obs_sf))], 'k--')
            
            obs_polygon_sf.append( plt.Polygon(obs[n].x_obs_sf, zorder=1))
            obs_polygon.append( plt.Polygon(obs[n].x_obs, zorder=2))
            if len(obstacleColor)==len(obs):
                obs_polygon_sf[n].set_color([1,1,1])
                obs_polygon[n].set_color(obstacleColor[n])
            else:
                obs_polygon_sf[n].set_color([1,1,1])
                obs_polygon[n].set_color(np.array([176,124,124])/255)
            
            plt.gca().add_patch(obs_polygon_sf[n])
            plt.gca().add_patch(obs_polygon[n])

            if show_obstacle_number:
                ax_ifd.annotate('{}'.format(n+1), xy=np.array(obs[n].x0)+0.16, textcoords='data', size=16, weight="bold")
            
            ax_ifd.plot(obs[n].x0[0],obs[n].x0[1],'k.')
            
            if hasattr(obs[n], 'center_dyn'):# automatic adaptation of center 
                ax_ifd.plot(obs[n].center_dyn[0],obs[n].center_dyn[1], 'k+', linewidth=18, markeredgewidth=4, markersize=13)

            if drawVelArrow and np.linalg.norm(obs[n].xd)>0:
                col=[0.5,0,0.9]
                fac=5 # scaling factor of velocity
                ax_ifd.arrow(obs[n].x0[0], obs[n].x0[1], obs[n].xd[0]/fac, obs[n].xd[1]/fac, head_width=0.3, head_length=0.3, linewidth=10, fc=col, ec=col, alpha=1)

    plt.gca().set_aspect('equal', adjustable='box')

    ax_ifd.set_xlim(x_range)
    ax_ifd.set_ylim(y_range)

    if noTicks:
        plt.tick_params(axis='both', which='major',bottom=False, top=False, left=False, right=False, labelbottom=False, labelleft=False)

    if showLabel:
        plt.xlabel(r'$\xi_1$', fontsize=16)
        plt.ylabel(r'$\xi_2$', fontsize=16)

    plt.tick_params(axis='both', which='major', labelsize=14)
    plt.tick_params(axis='both', which='minor', labelsize=12)

    ax_ifd.plot(xAttractor[0],xAttractor[1], 'k*',linewidth=18.0, markersize=18)

    # Show certain streamlines
    if np.array(points_init).shape[0]:
        plot_streamlines(points_init, ax_ifd, obs, xAttractor)

    if not draw_vectorField:
        plt.ion()
        plt.show()
        return

    start_time = time.time()

    # Create meshrgrid of points
    if type(point_grid)==int:
        N_x = N_y = point_grid
        YY, XX = np.mgrid[y_range[0]:y_range[1]:N_y*1j, x_range[0]:x_range[1]:N_x*1j]

    else:
        N_x = N_y = 1
        XX, YY = np.array([[point_grid[0]]]), np.array([[point_grid[1]]])

    if attractingRegion: # Forced to attracting Region
        def obs_avoidance_temp(x, xd, obs):
            return obs_avoidance_func(x, xd, obs, xAttractor)
        
        obs_avoidance= obs_avoidance_temp
    else:
        obs_avoidance = obs_avoidance_func
        
    xd_init = np.zeros((2,N_x,N_y))
    xd_mod  = np.zeros((2,N_x,N_y))

    for ix in range(N_x):
        for iy in range(N_y):
            pos = np.array([XX[ix,iy],YY[ix,iy]])
            xd_init[:,ix,iy] = dynamicalSystem(pos, x0=xAttractor) # initial DS
                
            xd_mod[:,ix,iy] = obs_avoidance(pos, xd_init[:,ix,iy], obs) # modulataed DS with IFD
    
    if sysDyn_init:
        fig_init, ax_init = plt.subplots(figsize=(5,2.5))
        res_init = ax_init.streamplot(XX, YY, xd_init[0,:,:], xd_init[1,:,:], color=[(0.3,0.3,0.3)])
        
        ax_init.plot(xAttractor[0],xAttractor[1], 'k*')
        plt.gca().set_aspect('equal', adjustable='box')

        plt.xlim(x_range)
        plt.ylim(y_range)

    indOfnoCollision = obs_check_collision_2d(obs, XX, YY)

    dx1_noColl = np.squeeze(xd_mod[0,:,:]) * indOfnoCollision
    dx2_noColl = np.squeeze(xd_mod[1,:,:]) * indOfnoCollision

    end_time = time.time()
    
    print('Number of points: {}'.format(point_grid*point_grid))
    print('Average time: {} ms'.format(np.round((end_time-start_time)/(N_x*N_y)*1000),5) )
    print('Modulation calulcation total: {} s'.format(np.round(end_time-start_time), 4))

    if plotStream:
        if colorCode:
            velMag = np.linalg.norm(np.dstack((dx1_noColl, dx2_noColl)), axis=2 )/6*100

            strm = res_ifd = ax_ifd.streamplot(XX, YY,dx1_noColl, dx2_noColl, color=velMag, cmap='winter', norm=matplotlib.colors.Normalize(vmin=0, vmax=10.) )
        else:
            # Normalize
            normVel = np.sqrt(dx1_noColl**2 + dx2_noColl**2)
            ind_nonZero = normVel>0
            dx1_noColl[ind_nonZero] = dx1_noColl[ind_nonZero]/normVel[ind_nonZero]
            dx2_noColl[ind_nonZero] = dx2_noColl[ind_nonZero]/normVel[ind_nonZero]

            res_ifd = ax_ifd.streamplot(XX, YY,dx1_noColl, dx2_noColl, color=streamColor, zorder=0)

        
    plt.ion()
    plt.show()

    returnFigure=True
    if returnFigure:
        return fig_ifd, ax_ifd
    
    if saveFigure:
        plt.savefig('fig/' + figName + '.eps', bbox_inches='tight')
        return fig_ifd, ax_ifd
