import numpy as np
from LaunchSiteData.judge_inside import judge_inside_border
from LaunchSiteData.judge_inside import judge_inside_circle
from LaunchSiteData.judge_inside import judge_inside_poly


class TaikiLand:
    def __init__(self):
        self.launch_point_LLH = np.array([42.514320, 143.439793, 0.0])
        self.points = []
        self.points.append([42.514340, 143.439894, 0.0])
        self.points.append([42.520564, 143.437342, 0.0])
        self.points.append([42.520939, 143.437916, 0.0])
        self.points.append([42.521376, 143.438037, 0.0])
        self.points.append([42.521869, 143.437817, 0.0])
        self.points.append([42.522543, 143.437870, 0.0])
        self.points.append([42.522637, 143.440032, 0.0])
        self.points.append([42.522326, 143.442730, 0.0])
        self.points.append([42.522068, 143.446304, 0.0])
        self.points.append([42.519429, 143.449155, 0.0])
        self.points.append([42.518901, 143.446610, 0.0])
        self.points.append([42.516320, 143.447509, 0.0])
        self.points.append([42.516170, 143.446469, 0.0])
        self.points.append([42.519668, 143.444919, 0.0])
        self.points.append([42.520032, 143.443932, 0.0])
        self.points.append([42.519811, 143.443460, 0.0])
        self.points.append([42.518823, 143.443395, 0.0])
        self.points.append([42.518657, 143.442861, 0.0])
        self.points.append([42.517423, 143.443108, 0.0])
        self.points.append([42.516827, 143.444292, 0.0])
        self.points.append([42.516107, 143.444170, 0.0])
        self.points.append([42.515663, 143.445209, 0.0])
        self.points.append([42.515714, 143.446207, 0.0])
        self.points.append([42.513618, 143.447081, 0.0])
        self.points.append([42.513342, 143.445773, 0.0])
        self.points.append([42.514024, 143.444927, 0.0])
        self.points.append([42.513465, 143.444155, 0.0])
        self.points.append([42.513221, 143.440440, 0.0])

    def in_range(self, landing_point_ENU):
        x = landing_point_ENU[0]
        y = landing_point_ENU[1]
        judge = judge_inside_poly([x, y], self.launch_point_LLH, self.points)
        return judge