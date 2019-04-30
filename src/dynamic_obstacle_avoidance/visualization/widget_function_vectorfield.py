from dynamic_obstacle_avoidance.dynamical_system.dynamical_system_representation import *
from dynamic_obstacle_avoidance.visualization.vector_field_visualization import *  #

saveFigures=False

def widget_ellipses_vectorfield(x1=2, x2=2, a1=1, a2=1, p1=1, p2=1, th_r=0, sf=1,
                                point_posX=4, point_posY=4,
                                x_low=0.8, x_high=4.2, y_low=-2, y_high=2, draw_vectorField=True):

    xlim=[x_low, x_high]
    ylim=[y_low, y_high]
    xAttractor = [0,0]

    obs = []
    x0=[x1, x2]
    a=[a1, a2]
    p=[p1,p2]
    th_r=th_r/180*pi
    vel = [0, 0]
    obs.append(Obstacle(a=a, p=p, x0=x0, th_r=th_r, sf=sf, xd=vel))

    point_pos = np.array([point_posX, point_posY])
    ds_init = point_pos - xAttractor

    ds_mod = obs_avoidance_interpolation_moving(point_pos, ds_init, obs)

    fig, ax = Simulation_vectorFields(xlim, ylim, point_grid=10, obs=obs, xAttractor=xAttractor, figName='linearSystem_avoidanceCircle', noTicks=False, figureSize=(13.,10), draw_vectorField=draw_vectorField)

    fig,ax  = plt.subplots() 
    # ax.quiver([point_pos[0]], [point_pos[1]], [ds_init[0]], [ds_init[1]], color='g', scale=5, zorder=10000)
    # ax.quiver([point_pos[0]], [point_pos[1]], [ds_mod[0]], [ds_mod[1]], color='r', scale=5, zorder=10)
    plt.show()
    # ax_init.quiver(point_pos[0], point_pos[1], ds_init[0], ds_init[1], c='b')

def widgetFunction_referencePoint(x1=2, x2=2, th_r=0,
                                refPoint_dir=0, refPoint_rat=0,
                                x_low=0.8, x_high=4.2, y_low=-2, y_high=2, draw_vectorField=True):
    
    x_lim=[x_low, x_high]
    y_lim=[y_low, y_high]
    
    xAttractor = [-12, 7.5]

    sf = 1.3
    a1, a2 = 6, 1.2
    p1, p2 = 1, 1

    obs = []
    x0=[x1, x2]
    a=[a1, a2]
    p=[p1,p2]
    th_r=th_r/180*pi
    vel = [0, 0]
    
    obs.append(Obstacle(a=a, p=p, x0=x0, th_r=th_r, sf=sf, xd=vel))

    refPoint_dir *= pi/180.
    obs[0].center_dyn = np.array([obs[0].a[0]*np.sqrt(refPoint_rat)*np.cos(refPoint_dir),
                                  obs[0].a[1]*np.sqrt(refPoint_rat)*np.sin(refPoint_dir)])
    rotationMatrix = np.array([[np.cos(th_r), np.sin(th_r)],
                               [-np.sin(th_r), np.cos(th_r)]])
    obs[0].center_dyn = rotationMatrix.T @ obs[0].center_dyn*obs[0].sf + obs[0].x0

    Simulation_vectorFields(x_lim, y_lim, point_grid=70, obs=obs, xAttractor=xAttractor, figName='linearSystem_avoidanceCircle', noTicks=False, figureSize=(13.,10), draw_vectorField=draw_vectorField )


def widgetFunction_intersection(center1_1=2, center2_1=2, orientation_1=0,
                                   center1_2=2, center2_2=2, orientation_2=0,
                                   center1_3=2, center2_3=2, orientation_3=0,
                                   center1_4=2, center2_4=2, orientation_4=0,
                                   # x1_1=2, x2_1=2, th_2=0,
                                   # x1=2, x2=2, th_r=0,
                                   # x1=2, x2=2, th_r=0,
                                   n_obstacles=2,
                                   x_low=0.8, x_high=4.2, y_low=-2, y_high=2, draw_vectorField=False):
    
    x_lim=[x_low, x_high]
    y_lim=[y_low, y_high]
    
    xAttractor = [12, 1]
    obs = []
    
    obs.append(Obstacle(a=[3,1], p=[1,1], x0=[center1_1,center2_1], th_r=orientation_1, sf=2.0))
    obs.append(Obstacle(a=[2,2], p=[1,1], x0=[center1_2,center2_2], th_r=orientation_2, sf=1.5))
    
    if n_obstacles>=3:
        obs.append(Obstacle(a=[1.0, 2.5], p=[3,3], x0=[center1_3,center2_3], th_r=orientation_3, sf=1.8))
    if n_obstacles>=4:
        obs.append(Obstacle(a=[0.8,1], p=[2,2], x0=[center1_4,center2_4], th_r=orientation_4, sf=3))

    Simulation_vectorFields(x_lim, y_lim, point_grid=70, obs=obs, xAttractor=xAttractor, figName='linearSystem_avoidanceCircle', noTicks=False, figureSize=(13.,10), draw_vectorField=draw_vectorField )



