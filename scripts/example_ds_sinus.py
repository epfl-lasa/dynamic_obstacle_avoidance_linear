import numpy as np
import matplotlib.pyplot as plt
from math import pi

def ds_sinus(pos, attractor_pos=[0, 1], sinus_magnitude=2,
             flip_curve=False, move_to_origin=False,
             maximum_velocity=0.5, y_scaling=2):
    ''' 2D dynamical system which follows sinus wave form

    Paramteter input
    position: np.array of shape (2.)
    x_position: float '''

    angle_attractor = np.arctan2(attractor_pos[1], attractor_pos[0])
    distance_attractor = np.linalg.norm(attractor_pos)

    cos_ang = np.cos(angle_attractor)
    sin_ang = np.sin(angle_attractor)
    rot_matrix = np.array([[cos_ang, sin_ang],
                           [-sin_ang, cos_ang]])

    pos = rot_matrix.dot(pos)

    desired_y = np.sin(pos[0]/distance_attractor*pi) * sinus_magnitude
    if flip_curve:
        desired_y *= (-1)

    vel = np.zeros(2)
    vel[1] = desired_y-pos[1]
        
    if move_to_origin:
        vel[0] = -pos[0]
    else:
        vel[0] = distance_attractor-pos[0]

    if  np.abs(vel[0]) > maximum_velocity:
        vel[0] = np.copysign(maximum_velocity, vel[0])


    vel[1] = vel[1]*y_scaling
        
    # Normalize
    norm_vel = np.linalg.norm(vel)
    if norm_vel > maximum_velocity:
        vel = vel/norm_vel * maximum_velocity
        
    vel = rot_matrix.T.dot(vel)
    
    return vel
    
dim = 2 # space dimension    
x_range = [-7, 7]
y_range = [-7, 7]

attractor_pos = np.array([3, 3])

n_grid = 20

# x_grid = np.linspace(x_range[0], x_range[1], n_grid)
# y_grid = np.linspace(y_range[0], y_range[1], n_grid)

num_x = num_y = n_grid
YY, XX = np.mgrid[y_range[0]:y_range[1]:num_y*1j, x_range[0]:x_range[1]:num_x*1j]

pos = np.dstack((XX, YY))
vel = np.zeros((n_grid, n_grid, dim))

for ix in range(n_grid):
    for iy in range(n_grid):
        vel[ix, iy, :] = ds_sinus(pos[ix, iy, :], attractor_pos,
                                  flip_curve=True,
                                  move_to_origin=True)
# Draw plot
plt.ion()
# fig, ax = plt.figure()
plt.figure()
plt.quiver(XX, YY, vel[:, :, 0], vel[:, :, 1])
plt.plot(attractor_pos[0], attractor_pos[1], 'k*', markeredgewidth=4, markersize=13)
plt.plot(0, 0, 'k.', markeredgewidth=4, markersize=13)
plt.savefig('../fig/example_DS_sinus.png')

print("The code finished correctly.")
