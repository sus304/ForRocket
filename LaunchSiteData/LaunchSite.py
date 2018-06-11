import numpy as np
import simplekml
from LaunchSiteData.coordinate import *


class LaunchSite:
    def magnetic_declination(self):
        lat = self.launch_point_LLH[0]
        lon = self.launch_point_LLH[1]

        delta_lat = lat - 37.0
        delta_lon = lon - 138.0
        mag_dec = (7.0 + 57.201 / 60.0)	+ (18.750 / 60.0) * delta_lat - (6.761 / 60.0) * delta_lon - (0.059 / 60.0) * delta_lat ** 2 - (0.014 / 60.0) * delta_lat * delta_lon - (0.579 / 60.0) * delta_lon ** 2
        return mag_dec
    
    def wind_law(self, alt, ref_wind_speed, ref_altitude):
        speed = ref_wind_speed * (alt / ref_altitude) ** (1.0 / self.wind_power_exp)
        return speed

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

    def plot_kml(self, landing_points_ENU, result_dir, name=''):
        kml = simplekml.Kml()
        for vel in landing_points_ENU:
            landing_points_LLH = [ENU2LLH(self.launch_point_LLH, np.append(point, 0.0)) for point in vel]
            landing_points_LLH.append(landing_points_LLH[0])
            linestring = kml.newlinestring()
            linestring.style.linestyle.color = simplekml.Color.red
            linestring.coords = landing_points_LLH
        kml.save(result_dir + '/' + name + 'landing_range.kml')


