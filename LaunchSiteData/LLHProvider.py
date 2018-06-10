import numpy as np
import matplotlib.pyplot as plt
import re
import simplekml

from LaunchSiteData.Noshiro_land_3rd import NoshiroAsanai3rd
from LaunchSiteData.Noshiro_sea import NoshiroOchiai3km
from LaunchSiteData.Taiki_land import TaikiLand


class LaunchSite:
    def __init__(self, launch_site_name, result_dir):
        self.result_dir = result_dir
        name = launch_site_name
        noshiro = True if re.search(r'noshiro|nosiro', name, re.IGNORECASE) else False
        taiki = True if re.search(r'taiki', name, re.IGNORECASE) else False
        land = True if re.search(r'land', name, re.IGNORECASE) else False
        sea = True if re.search(r'sea', name, re.IGNORECASE) else False
        asanai = True if re.search(r'asanai|asauchi|asauti', name, re.IGNORECASE) else False
        field3rd = True if re.search(r'3rd', name, re.IGNORECASE) else False
        ochiai = True if re.search(r'ochiai|otiai', name, re.IGNORECASE) else False

        if noshiro and (land or asanai or field3rd):
            self.site = NoshiroAsanai3rd()
        elif noshiro and (sea or oshiai):
            self.site = NoshiroOchiai3km()
        elif taiki:
            self.site = TaikiLand()

    def magnetic_declination(self):
        lat = self.site.launch_point_LLH[0]
        lon = self.site.launch_point_LLH[1]

        delta_lat = lat - 37.0
        delta_lon = lon - 138.0
        mag_dec = (7.0 + 57.201 / 60.0)	+ (18.750 / 60.0) * delta_lat - (6.761 / 60.0) * delta_lon - (0.059 / 60.0) * delta_lat ** 2 - (0.014 / 60.0) * delta_lat * delta_lon - (0.579 / 60.0) * delta_lon ** 2
        return mag_dec

    def plot_kml(self, landing_points_ENU, name=''):
        coord = Coordinate()
        kml = simplekml.Kml()
        for vel in landing_points_ENU:
            landing_points_LLH = [coord.ENU2LLH(self.site.launch_point_LLH, np.append(point, 0.0)) for point in vel]
            landing_points_LLH.append(landing_points_LLH[0])
            linestring = kml.newlinestring()
            linestring.style.linestyle.color = simplekml.Color.red
            linestring.coords = landing_points_LLH
        kml.save(self.result_dir + '/' + name + 'landing_range.kml')

    def points_in_range(self, landing_points_ENU):
        verify_list = []
        for vel in landing_points_ENU:
            verify_list_vel = []
            verify_list_vel = [self.site.in_range(point) for point in vel]
            verify_list.append(verify_list_vel)
        verify_array = np.array(verify_list)
        # all True対策
        verify_array = np.append(verify_array, np.array([[False] * len(verify_array[0, :])]), axis=0)

        limit_index_array = np.empty(0)
        for i in range(len(verify_array[0, :])):
            limit_index = int(np.argmin(verify_array[:, i])) - 1
            limit_index_array = np.append(limit_index_array, limit_index)
        return limit_index_array

    def wind_limit(self, vel_wind_array, angle_wind_array, hard_landing_points, soft_landing_points):
        self.plot_kml(hard_landing_points, name='hard')
        self.plot_kml(soft_landing_points, name='soft')
        limit_index_hard_array = self.points_in_range(hard_landing_points)
        limit_index_soft_array = self.points_in_range(soft_landing_points)
        wind_limit_index_array = np.array([int(min(hard, soft)) for hard, soft in zip(limit_index_hard_array, limit_index_soft_array)])
        vel_wind_array = np.append(vel_wind_array, 0.0)

        txt = open(self.result_dir + '/wind_limit.txt', mode='w')
        txt.write('Wind Limit\n')
        for i in range(len(wind_limit_index_array)):
            txt.writelines([str(angle_wind_array[i]), ' [deg]: ', str(vel_wind_array[wind_limit_index_array[i]]), ' [m/s]\n'])
        txt.close()


class Judge_inside_circle:
    def __init__(self, launch_point_LLH, center_point_LLH, radius):
        # LLH : [latitude, longitude, height] = [deg, deg, m]
        coord = Coordinate()
        self.center_point = coord.LLH2ENU(launch_point_LLH, center_point_LLH)
        self.radius = radius

    def __call__(self, landing_point):
        x = landing_point[0]
        y = landing_point[1]
        distance = (x - self.center_point[0]) ** 2 + (y - self.center_point[1])
        judge = True if distance < self.radius**2 else False
        return judge


class Judge_inside_border:
    def __init__(self, launch_point_LLH, edge_point1_LLH, edge_point2_LLH, over_axis=[1,-1]):
        # LLH : [latitude, longitude, height] = [deg, deg, m]
        # over axis: False判定の方向をx,yの単位ベクトルで
        # ENUでE正N負がNG=overなら[1,-1]
        coord = Coordinate()
        edge_point1 = coord.LLH2ENU(launch_point_LLH, edge_point1_LLH)
        edge_point2 = coord.LLH2ENU(launch_point_LLH, edge_point2_LLH)
        self.over_axis = np.array(over_axis)

        dx = (edge_point2[0] - edge_point1[0])
        dy = (edge_point2[1] - edge_point1[1])
        self.slope = dy / dx if dx != 0 else 0
        self.intercept_y_border = (edge_point2[0] * edge_point1[1] - edge_point1[0] * edge_point2[1]) / dx

    def __call__(self, landing_point):
        x = landing_point[0]
        y = landing_point[1]
        move_y = y - self.intercept_y_border
        intercept_y_landing = self.intercept_y_border + move_y - self.slope * x

        judge = True if intercept_y_landing * self.over_axis[1] < self.intercept_y_border * self.over_axis[1] else False
        return judge


class Judge_inside_poly:
    def __init__(self, launch_point_LLH, poly_points_LLH):
        coord = Coordinate()
        poly_points_LLH = poly_points_LLH
        if poly_points_LLH[0] == poly_points_LLH[-1]:
            pass
        else:
            poly_points_LLH.append(poly_points_LLH[0])
        self.poly_points = np.array([coord.LLH2ENU(launch_point_LLH, poly_point_LLH) for poly_point_LLH in poly_points_LLH])


    def __call__(self, landing_point):
        x = landing_point[0]
        y = landing_point[1]
        cross = 0
        for p1, p2 in zip(self.poly_points[:-1], self.poly_points[1:]):
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


class Coordinate:
    def LLH2ECEF(self, LLH):
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

    def LLH2ENU(self, launch_LLH, point_LLH):
        # LLH : [latitude, longitude, height] = [deg, deg, m]
        lat = np.deg2rad(launch_LLH[0])
        lon = np.deg2rad(launch_LLH[1])
        DCM_ECEF2NED = np.array([[-np.sin(lat) * np.cos(lon), -np.sin(lat) * np.sin(lon), np.cos(lat)],
                                [-np.sin(lon)              , np.cos(lon)               , 0.0         ],
                                [-np.cos(lat) * np.cos(lon), -np.cos(lat) * np.sin(lon), -np.sin(lat)]])

        launch_ECEF = self.LLH2ECEF(launch_LLH)
        point_ECEF = self.LLH2ECEF(point_LLH)
        point_ECEF -= launch_ECEF

        point_NED = DCM_ECEF2NED.dot(point_ECEF)
        point_ENU = [point_NED[1], point_NED[0], -point_NED[2]]

        return point_ENU

    def ENU2LLH(self, launch_LLH, point_ENU):
        lat = np.deg2rad(launch_LLH[0])
        lon = np.deg2rad(launch_LLH[1])
        height = launch_LLH[2]
        launch_ECEF = self.LLH2ECEF(launch_LLH)
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

# def kml_make(name,Launch_LLH):
#   import simpelkml
#   Log = np.loadtxt('Position_log.csv',delimiter=",",skiprows=1)
#   array = Log[:,1]
#   array_len = len(array)
#   print ":"
#   Position_ENU = np.zeros((array_len,3))
#   Position_ENU[:,0] = np.array(Log[:,0])
#   Position_ENU[:,1] = np.array(Log[:,1])
#   Position_ENU[:,2] = np.array(Log[:,2])
  
#   Position_ecef = np.zeros((array_len,3))
#   Position_LLH = np.zeros((array_len,3))
#   print ":"
#   for i in range(array_len):
#     Position_ecef[i,:] = ENU2ECEF(Position_ENU[i,:],Launch_LLH)
#     Position_LLH[i,:] = ECEF2LLH(Position_ecef[i,:])
#   print ":"
  
#   header = 'Latitude,Longitude,Height'
#   np.savetxt("Result Log 1.csv",Position_LLH,fmt = '%.5f',delimiter = ',',header = header)
  
#   kml = simplekml.Kml(open=1)
#   Log_LLH = []
#   for i in range(array_len):
#     if 0 == i % 10000:
#       Log_LLH.append((Position_LLH[i,1],Position_LLH[i,0],Position_LLH[i,2]))
#   print ":"
#   line = kml.newlinestring(name = name)
#   line.style.linestyle.width = 5
#   line.style.linestyle.color = simplekml.Color.red
#   line.extrude = 1
#   line.altitudemode = simplekml.AltitudeMode.absolute
#   line.coords = Log_LLH
#   line.style.linestyle.colormode = simplekml.ColorMode.random
#   kml.save(name + ".kml")


if __name__ == '__main__':
    site = LaunchSite('taiki')
    print(site.launch_point_LLH)

    