import numpy as np
from LaunchSiteData.judge_inside import judge_inside_border
from LaunchSiteData.judge_inside import judge_inside_circle
from LaunchSiteData.judge_inside import judge_inside_poly
from LaunchSiteData.LaunchSite import LaunchSite


class NoshiroOchiai(LaunchSite):
    def __init__(self, range_diameter):
        self.name = 'Noshiro Ochiai'
        self.launch_point_LLH = np.array([40.242865, 140.010450, 0.0])
        self.center_point_LLH = np.array([40.245567, 139.993297, 0.0])
        self.radius = range_diameter * 0.5
        self.wind_power_exp = 6.0        
        # self.wind_power_exp = 3.1        

    def in_range(self, landing_point_ENU):
        x = landing_point_ENU[0]
        y = landing_point_ENU[1]
        judge = judge_inside_circle([x, y], self.launch_point_LLH, self.center_point_LLH, self.radius) and judge_inside_border([x, y], self.launch_point_LLH, np.array([40.243015, 140.007566, 0.0]), np.array([40.235585, 140.005619, 0.0]), [1, -1])
        return judge

    