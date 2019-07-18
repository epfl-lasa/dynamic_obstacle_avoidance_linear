#!/USSR/bin/python3

'''
Script which creates a variety of examples of local modulation of a vector field with obstacle avoidance. 

@author LukasHuber
@date 2018-02-15
'''

# Command to automatically reload libraries -- in ipython before exectureion
import numpy as np
import matplotlib.pyplot as plt

# Custom libraries
from dynamic_obstacle_avoidance.dynamical_system.dynamical_system_representation import *
from dynamic_obstacle_avoidance.visualization.vector_field_visualization import *  #
from dynamic_obstacle_avoidance.obstacle_avoidance.obstacle import *

########################################################################
# Chose the option you want to run as a number in the option list (integer from -2 to 10)
options = [0]

N_resol = 100

saveFigures=False

########################################################################

def main(options=[], N_resol=100, saveFigures=False):
    for option in options:
        obs = [] # create empty obstacle list
        if option==0:
            x_lim = [-0.5,10]
            y_lim = [-5,5]

            xAttractor=[1,0]
            
            # cuboid_obs = Cuboid(
            #     axes_length=[3, 3],
            #     center_position=[1, 0],
            #     orientation=0./180*pi,
            #     absolut_margin=0.0)
            
            obs.append(Ellipse(axes_length=[2,4], center_position=[4,0],
                                p=[1,1], orientation=0, sf=1))
            
            # def ds_init_temp(x, x0):
                # return linearDS_constVel(x, x0, A=np.array([[1,-4],[2,1]]))

            # fig_mod, ax_mod = Simulation_vectorFields(x_lim, y_lim,  obs=obs, xAttractor=xAttractor, saveFigure=saveFigures, figName='linearSystem_boundaryEllipse', noTicks=False, draw_vectorField=True,  automatic_reference_point=False, point_grid=10, dynamicalSystem=ds_init_temp)
            
            fig_mod, ax_mod = Simulation_vectorFields(x_lim, y_lim,  obs=obs, xAttractor=xAttractor, saveFigure=saveFigures, figName='linearSystem_boundaryEllipse', noTicks=False, draw_vectorField=True,  automatic_reference_point=False, point_grid=10)

            
if (__name__==("__main__")):
    if len(sys.argv) > 1:
        options = sys.argv[1]

    if len(sys.argv) > 2:
        N_resol = sys.argv[2]

    if len(sys.argv) > 3:
        saveFigures = sys.argv[3]

    main(options=options, N_resol=N_resol, saveFigures=saveFigures)

    # input("\nPress enter to continue...")

# Run function

