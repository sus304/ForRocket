import numpy as np

def judge_inside_circle(landing_point, launch_point_LLH, center_point_LLH, radius):
    # LLH : [latitude, longitude, height] = [deg, deg, m]
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


def LLH2ECEF(LLH):
    # LLH : [latitude, longitude, height] = [deg, deg, m]
    lat = LLH[0]
    lon = LLH[1]
    height = LLH[2]

    # WGS84 Constant
    a = 6378137.0
    f = 1.0 / 298.257223563
    # e_sq = f * (2.0 - f)
    e_sq = 0.0818191908426 ** 2
    N = a / np.sqrt(1.0 - e_sq * np.power(np.sin(np.radians(lat)), 2))
    point_ECEF = np.zeros(3)
    point_ECEF[0] = (N + height) * np.cos(np.radians(lat)) * np.cos(np.radians(lon))
    point_ECEF[1] = (N + height) * np.cos(np.radians(lat)) * np.sin(np.radians(lon))
    point_ECEF[2] = (N * (1.0 - e_sq) + height) * np.sin(np.radians(lat))
    return point_ECEF

def LLH2ENU(launch_LLH, point_LLH):
    # LLH : [latitude, longitude, height] = [deg, deg, m]
    lat = np.deg2rad(launch_LLH[0])
    lon = np.deg2rad(launch_LLH[1])
    DCM_ECEF2NED = np.array([[-np.sin(lat) * np.cos(lon), -np.sin(lat) * np.sin(lon), np.cos(lat)],
                            [-np.sin(lon)              , np.cos(lon)               , 0.0         ],
                            [-np.cos(lat) * np.cos(lon), -np.cos(lat) * np.sin(lon), -np.sin(lat)]])

    launch_ECEF = LLH2ECEF(launch_LLH)
    point_ECEF = LLH2ECEF(point_LLH)
    point_ECEF -= launch_ECEF

    point_NED = DCM_ECEF2NED.dot(point_ECEF)
    point_ENU = [point_NED[1], point_NED[0], -point_NED[2]]

    return point_ENU

def ENU2LLH(launch_LLH, point_ENU):
    lat = np.deg2rad(launch_LLH[0])
    lon = np.deg2rad(launch_LLH[1])
    height = launch_LLH[2]
    launch_ECEF = LLH2ECEF(launch_LLH)
    e = point_ENU[0]
    n = point_ENU[1]
    u = point_ENU[2]

    x_ecef = -np.sin(lat) * np.cos(lon) * n - np.sin(lon) * e - np.cos(lat) * np.cos(lon) * (-u) + launch_ECEF[0]
    y_ecef = -np.sin(lat) * np.sin(lon) * n + np.cos(lon) * e - np.cos(lat) * np.sin(lon) * (-u) + launch_ECEF[1]
    z_ecef = np.cos(lat) * n - np.sin(lat) * (-u) + launch_ECEF[2]

    # WGS84 Constant
    a = 6378137.0
    f = 1.0 / 298.257223563
    b = a * (1.0 - f)
    e_sq = 2.0 * f - (f * f)
    e2_sq = (e_sq * a * a) / (b * b)

    p = np.sqrt(x_ecef ** 2 + y_ecef ** 2)
    theta = np.arctan2(z_ecef * a, p * b)
    point_LLH = np.zeros(3)
    point_LLH[0] = np.degrees(np.arctan2(z_ecef + e2_sq * b * np.power(np.sin(theta), 3),p - e_sq * a * np.power(np.cos(theta), 3)))
    point_LLH[1] = np.degrees(np.arctan2(y_ecef, x_ecef))
    N = a / np.sqrt(1.0 - e_sq * np.power(np.sin(np.radians(point_LLH[0])), 2))
    point_LLH[2] = (p / np.cos(np.radians(point_LLH[0]))) - N
    point_earth = [point_LLH[1], point_LLH[0], point_LLH[1]]

    return point_earth
