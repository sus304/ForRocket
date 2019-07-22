import numpy as np

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
