import numpy as np
import pymap3d as pm
from LaunchSiteData.coordinate import *

def judge_inside_circle(landing_point, launch_point_LLH, center_point_LLH, radius):
    # LLH : [latitude, longitude, height] = [deg, deg, m]
    center_point = pm.geodetic2enu
    center_point = LLH2ENU(launch_point_LLH, center_point_LLH)
    x = landing_point[0]
    y = landing_point[1]
    distance = (x - center_point[0]) ** 2 + (y - center_point[1])
    judge = True if distance < radius**2 else False
    return judge



def judge_inside_border(landing_point, launch_point_LLH, edge_point1_LLH, edge_point2_LLH, over_axis=[1,-1]):
    # LLH : [latitude, longitude, height] = [deg, deg, m]
    # over axis: False判定の方向をx,yの単位ベクトルで
    # ENUでE正N負がNG=overなら[1,-1]
    edge_point1 = LLH2ENU(launch_point_LLH, edge_point1_LLH)
    edge_point2 = LLH2ENU(launch_point_LLH, edge_point2_LLH)
    over_axis = np.array(over_axis)

    dx = (edge_point2[0] - edge_point1[0])
    dy = (edge_point2[1] - edge_point1[1])
    slope = dy / dx if dx != 0 else 0
    intercept_y_border = (edge_point2[0] * edge_point1[1] - edge_point1[0] * edge_point2[1]) / dx

    x = landing_point[0]
    y = landing_point[1]
    move_y = y - intercept_y_border
    intercept_y_landing = intercept_y_border + move_y - slope * x

    judge = True if intercept_y_landing * over_axis[1] < intercept_y_border * over_axis[1] else False
    return judge


def judge_inside_poly(landing_point, launch_point_LLH, poly_points_LLH):
    if poly_points_LLH[0] == poly_points_LLH[-1]:
        pass
    else:
        poly_points_LLH.append(poly_points_LLH[0])
    poly_points = np.array([LLH2ENU(launch_point_LLH, poly_point_LLH) for poly_point_LLH in poly_points_LLH])


    x = landing_point[0]
    y = landing_point[1]
    cross = 0
    for p1, p2 in zip(poly_points[:-1], poly_points[1:]):
        max_x = max(p1[0], p2[0])
        max_y = max(p1[1], p2[1])
        min_y = min(p1[1], p2[1])

        if y == p1[1]:
            if x < p1[0]:
                cross += 1
        elif min_y <= y <= max_y and x < max_x:
            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]
            if dx == 0:
                if x <= max_x:
                    cross += 1
            elif dy == 0:
                pass
            else:
                slope = dy / dx
                intercept_y = p1[1] - slope * p1[0]
                x_cross = (y - intercept_y) / slope
                if x < x_cross:
                    cross += 1
    judge = False if cross % 2 == 0 else True
    return judge

    def in_range(self, landing_point_ENU):
        x = landing_point_ENU[0]
        y = landing_point_ENU[1]
        judge = judge_inside_poly([x, y], self.launch_point_LLH, [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
        return judge

    def points_in_range(self, landing_points_ENU):
        verify_list = []
        for vel in landing_points_ENU:
            verify_list_vel = []
            verify_list_vel = [self.in_range(point) for point in vel]
            verify_list.append(verify_list_vel)
        verify_array = np.array(verify_list)
        # all True対策
        verify_array = np.append(verify_array, np.array([[False] * len(verify_array[0, :])]), axis=0)

        limit_index_array = np.empty(0)
        for i in range(len(verify_array[0, :])):
            limit_index = int(np.argmin(verify_array[:, i])) - 1
            limit_index_array = np.append(limit_index_array, limit_index)
        return limit_index_array

    def wind_limit(self, vel_wind_array, angle_wind_array, hard_landing_points, soft_landing_points, result_dir):
        limit_index_hard_array = self.points_in_range(hard_landing_points)
        limit_index_soft_array = self.points_in_range(soft_landing_points)
        wind_limit_index_array = np.array([int(min(hard, soft)) for hard, soft in zip(limit_index_hard_array, limit_index_soft_array)])
        vel_wind_array = np.append(vel_wind_array, 0.0)

        txt = open(result_dir + '/wind_limit.txt', mode='w')
        txt.write('Wind Limit\n')
        for i in range(len(wind_limit_index_array)):
            txt.writelines([str(angle_wind_array[i]), ' [deg]: ', str(vel_wind_array[wind_limit_index_array[i]]), ' [m/s]\n'])
        txt.close()