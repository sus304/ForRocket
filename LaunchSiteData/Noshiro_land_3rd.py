import numpy as np
from LaunchSiteData.judge_inside import judge_inside_border
from LaunchSiteData.judge_inside import judge_inside_circle
from LaunchSiteData.judge_inside import judge_inside_poly
from LaunchSiteData.LaunchSite import LaunchSite


class NoshiroAsanai(LaunchSite):
    def __init__(self):
        self.name = 'Noshiro Asanai'
        self.launch_point_LLH = np.array([40.138633, 139.984850, 0.0])
        self.wind_power_exp = 4.5        

        # 保安範囲
        self.points = []
        # ギリギリVer
        # self.points.append([40.139816, 139.983804, 0.0])
        # self.points.append([40.137125, 139.982444, 0.0])
        # self.points.append([40.135588, 139.981298, 0.0])
        # self.points.append([40.134917, 139.981260, 0.0])
        # self.points.append([40.134614, 139.981351, 0.0])
        # self.points.append([40.134459, 139.981516, 0.0])
        # self.points.append([40.134877, 139.982257, 0.0])
        # self.points.append([40.135198, 139.982698, 0.0])
        # self.points.append([40.135512, 139.983439, 0.0])
        # self.points.append([40.136940, 139.984687, 0.0])
        # self.points.append([40.137521, 139.985506, 0.0])
        # self.points.append([40.137521, 139.985506, 0.0])
        # ちょっと狭めで安全側
        self.points.append([40.139725, 139.983939, 0.0])
        self.points.append([40.136127, 139.982133, 0.0])
        self.points.append([40.135607, 139.981753, 0.0])
        self.points.append([40.134911, 139.981451, 0.0])
        self.points.append([40.134821, 139.981692, 0.0])
        self.points.append([40.135639, 139.983324, 0.0])
        self.points.append([40.137052, 139.984608, 0.0])
        self.points.append([40.138053, 139.985781, 0.0])
        self.points.append([40.139075, 139.986297, 0.0])

    def in_range(self, landing_point_ENU):
        x = landing_point_ENU[0]
        y = landing_point_ENU[1]
        judge = judge_inside_poly([x, y], self.launch_point_LLH, self.points)
        return judge
        
