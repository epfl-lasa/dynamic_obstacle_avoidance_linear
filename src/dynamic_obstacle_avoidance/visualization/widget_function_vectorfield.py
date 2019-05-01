from dynamic_obstacle_avoidance.dynamical_system.dynamical_system_representation import *
from dynamic_obstacle_avoidance.visualization.vector_field_visualization import *  #

from ipywidgets import interact, interactive, fixed, interact_manual
from ipywidgets import FloatSlider, IntSlider
import ipywidgets as widgets

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
                                x_low=0.8, x_high=4.2, y_low=-2, y_high=2, draw_style="None"):
    
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

    if draw_style=="Simulated streamline":
        n_points = 6
        points_init = np.vstack((np.ones(n_points)*x_lim[1],
                                 np.linspace(y_lim[0], y_lim[1], n_points)))
        points_init = points_init[:, 1:-1]
        Simulation_vectorFields(x_lim, y_lim, point_grid=70, obs=obs, xAttractor=xAttractor, figName='linearSystem_avoidanceCircle', noTicks=False, figureSize=(13.,10), draw_vectorField=False, points_init=points_init)
        
    elif draw_style=="Vectorfield":
        Simulation_vectorFields(x_lim, y_lim, point_grid=70, obs=obs, xAttractor=xAttractor, figName='linearSystem_avoidanceCircle', noTicks=False, figureSize=(13.,10), draw_vectorField=True )
    else:
        Simulation_vectorFields(x_lim, y_lim, point_grid=70, obs=obs, xAttractor=xAttractor, figName='linearSystem_avoidanceCircle', noTicks=False, figureSize=(13.,10), draw_vectorField=False)



class WidgetClass_intersection():
    def __init__(self,  x_lim, y_lim, xAttractor=[12,0]):
        self.obs = []
        self.obs.append(Obstacle(a=[3,1], p=[1,1],
                                 x0=[-14,4], th_r=45/180*pi, sf=2.0))
        self.obs.append(Obstacle(a=[3,1], p=[1,1],
                                 x0=[-3, 10], th_r=0/180*pi, sf=1.3))
        self.obs.append(Obstacle(a=[2.4, 2.4], p=[4,4],
                                 x0=[-6, 4], th_r=-80/180*pi, sf=1.1))
        self.obs.append(Obstacle(a=[3,2], p=[1,2],
                                 x0=[10, 14], th_r=-110/180*pi, sf=1.5))

        self.x_lim = x_lim
        self.y_lim = y_lim
        self.xAttractor = xAttractor
        
    def set_obstacle_number(self, n_obstacles=2):
        self.n_obstacles = n_obstacles

    def set_obstacle_values(self, it_obs, x0_1, x0_2, th_r):
        it_obs -= 1
        self.obs[it_obs].x0 = [x0_1, x0_2]
        self.obs[it_obs].th_r = th_r
        self.obs[it_obs]  = Obstacle(x0=[x0_1, x0_2], th_r=th_r,
                                     a=self.obs[it_obs].a, p=self.obs[it_obs].p, sf=self.obs[it_obs].sf)
        
    def update(self, check_vectorfield=True):
        obs_cp = self.obs[:self.n_obstacles]
        Simulation_vectorFields(self.x_lim, self.y_lim, point_grid=70, obs=obs_cp, xAttractor=self.xAttractor, figName='linearSystem_avoidanceCircle', noTicks=False, figureSize=(13.,10), draw_vectorField=check_vectorfield, show_obstacle_number=True)


def run_obstacle_description():
    #%matplotlib gtk
    x_lim = [-16, 16]
    y_lim = [-2, 18]

    x1_widget = FloatSlider(description='Position \( x_1\)', min=x_lim[0], max=x_lim[1], step=0.1, value=6)
    x2_widget = FloatSlider(description='Position \( x_2\)', min=y_lim[0], max=y_lim[1], step=0.1, value=8)

    axis_widget1 = FloatSlider(description='Axis length 1', min=0.1, max=8, step=0.1, value=5)
    axis_widget2 = FloatSlider(description='Axis length 2', min=0.1, max=8, step=0.1, value=3)

    curvature_widget1 = IntSlider(description='Curvature 1', min=1, max=5, value=3)
    curvature_widget2 = IntSlider(description='Curvature 2', min=1, max=5, value=1)

    margin_widget = FloatSlider(description='Safety Margin', min=1, max=3, step=0.1, value=1.2)

    angle_widget = FloatSlider(description='Orientation', min=-180, max=180, step=1, value=30)

    pointX_widget = FloatSlider(description='Point position x', min=x_lim[0], max=x_lim[1], step=0.1, value=-3)
    pointY_widget = FloatSlider(description='Point position y', min=y_lim[0], max=y_lim[1], step=0.1, value=15)

    print("Change parameters and press <<Run Interact>> to apply.")

    interact_manual(widget_ellipses_vectorfield, x1=x1_widget, x2=x2_widget,
                    th_r=angle_widget,
                    a1=axis_widget1, a2=axis_widget2,
                    p1=curvature_widget1, p2=curvature_widget2,
                    sf=margin_widget,
                    draw_vectorField=True,
                    point_posX=pointX_widget, point_posY=pointY_widget,
                    x_low=fixed(x_lim[0]), x_high=fixed(x_lim[1]), 
                    y_low=fixed(y_lim[0]), y_high=fixed(y_lim[1]));
