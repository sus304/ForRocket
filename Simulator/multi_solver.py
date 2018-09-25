import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import simplekml
import pymap3d as pm
import pymap3d.vincenty

import Simulator.coordinate as coord

    
    def output_kml(self, landing_points_ENU, result_dir, name=''):
        kml = simplekml.Kml()
        for vel in landing_points_ENU:
            landing_points_LLH = [ENU2LLH(self.launch_point_LLH, np.append(point, 0.0)) for point in vel]
            landing_points_LLH.append(landing_points_LLH[0])
            linestring = kml.newlinestring()
            linestring.style.linestyle.color = simplekml.Color.red
            linestring.coords = landing_points_LLH
        kml.save(result_dir + '/' + name + 'landing_range.kml')
    

import os
import gc
from tqdm import tqdm

class WindMapSolver:
    def __init__(self, vel_wind_config, angle_wind_config, single_solver, result_dir):
        self.result_dir = result_dir
        self.single_solver = single_solver

        Vel_wind_min = vel_wind_config[0]
        Vel_wind_max = vel_wind_config[1]
        Vel_wind_step = vel_wind_config[2]
        angle_wind_min = angle_wind_config[0]
        angle_wind_max = angle_wind_config[1]
        angle_wind_step = angle_wind_config[2]

        self.Vel_wind_array = np.arange(Vel_wind_min, Vel_wind_max + Vel_wind_step, Vel_wind_step)
        self.angle_wind_array = np.arange(angle_wind_min, angle_wind_max + angle_wind_step, angle_wind_step)

        self.header = (self.angle_wind_array)
        self.index = (self.Vel_wind_array)

    def solve_map(self, rocket):
        time_apogee_array = np.empty((0, len(self.angle_wind_array)))
        Vel_air_apogee_array = np.empty((0, len(self.angle_wind_array)))
        altitude_apogee_array = np.empty((0, len(self.angle_wind_array)))
        downrange_hard_array = np.empty((0, len(self.angle_wind_array)))
        downrange_soft_array = np.empty((0, len(self.angle_wind_array)))
        Vel_air_max_array = np.empty((0, len(self.angle_wind_array)))
        Mach_max_array = np.empty((0, len(self.angle_wind_array)))
        MaxQ_array = np.empty((0, len(self.angle_wind_array)))
        time_hard_landing_array = np.empty((0, len(self.angle_wind_array)))
        hard_landing_points = []
        time_sepa2_array = np.empty((0, len(self.angle_wind_array)))
        time_soft_landing_array = np.empty((0, len(self.angle_wind_array)))
        soft_landing_points = []
        for Vel_wind in tqdm(self.Vel_wind_array):
            time_apogee_array_angle = np.empty(0)
            Vel_air_apogee_array_angle = np.empty(0)
            altitude_apogee_array_angle = np.empty(0)
            downrange_hard_array_angle = np.empty(0)
            downrange_soft_array_angle = np.empty(0)
            Vel_air_max_array_angle = np.empty(0)
            Mach_max_array_angle = np.empty(0)
            MaxQ_array_angle = np.empty(0)
            time_hard_landing_array_angle = np.empty(0)
            hard_landing_points_angle = []
            time_sepa2_array_angle = np.empty(0)
            time_soft_landing_array_angle = np.empty(0)
            soft_landing_points_angle = []
            for angle_wind in self.angle_wind_array:
                # run solver
                self.single_solver.result_dir = self.result_dir + '/Result_single_' + str(int(Vel_wind)) + str(int(angle_wind))
                os.mkdir(self.single_solver.result_dir)
                if self.single_solver.wind_file_exist:
                    alt_array = self.single_solver.wind_array[:, 0]
                else:
                    alt_array = np.arange(0.0, 20005.0, 5.0)
                    wind_speed_array = self.single_solver.launch_site.wind_law(alt_array, Vel_wind, self.single_solver.ref_alt)
                    self.single_solver.launch_site.wind_speed = interpolate.interp1d(alt_array, wind_speed_array, kind='linear', bounds_error=False, fill_value=(wind_speed_array[0], wind_speed_array[-1]))
                wind_direction_array = np.array([angle_wind] * alt_array.size)
                self.single_solver.launch_site.wind_direction = interpolate.interp1d(alt_array, wind_direction_array + self.single_solver.launch_site.magnetic_declination(), kind='linear', bounds_error=False, fill_value=(wind_direction_array[0], wind_direction_array[-1]))
                self.single_solver.solve_dynamics(rocket)

                # Result Pickup
                time_apogee_array_angle = np.append(time_apogee_array_angle, self.single_solver.time_apogee)
                Vel_air_apogee_array_angle = np.append(Vel_air_apogee_array_angle, self.single_solver.Vel_air_abs_apogee)
                altitude_apogee_array_angle = np.append(altitude_apogee_array_angle, self.single_solver.altitude_apogee)
                downrange_hard_array_angle = np.append(downrange_hard_array_angle, self.single_solver.downrange_hard_landing)
                downrange_soft_array_angle = np.append(downrange_soft_array_angle, self.single_solver.downrange_soft_landing)
                Vel_air_max_array_angle = np.append(Vel_air_max_array_angle, self.single_solver.Vel_air_max)
                Mach_max_array_angle = np.append(Mach_max_array_angle, self.single_solver.Mach_max)
                MaxQ_array_angle = np.append(MaxQ_array_angle, self.single_solver.maxQ)
                time_hard_landing_array_angle = np.append(time_hard_landing_array_angle, self.single_solver.time_hard_landing)
                hard_landing_points_angle.append(self.single_solver.hard_landing_point)
                time_sepa2_array_angle = np.append(time_sepa2_array_angle, self.single_solver.time_sepa2)
                time_soft_landing_array_angle = np.append(time_soft_landing_array_angle, self.single_solver.time_soft_landing)
                soft_landing_points_angle.append(self.single_solver.soft_landing_point)

                gc.collect()  # ガベージコレクションの開放、メモリクリアになってくれる？

            time_apogee_array = np.append(time_apogee_array, np.array([time_apogee_array_angle]), axis=0)
            Vel_air_apogee_array = np.append(Vel_air_apogee_array, np.array([Vel_air_apogee_array_angle]), axis=0)
            altitude_apogee_array = np.append(altitude_apogee_array, np.array([altitude_apogee_array_angle]), axis=0)
            downrange_hard_array = np.append(downrange_hard_array, np.array([downrange_hard_array_angle]), axis=0)
            downrange_soft_array = np.append(downrange_soft_array, np.array([downrange_soft_array_angle]), axis=0)
            Vel_air_max_array = np.append(Vel_air_max_array, np.array([Vel_air_max_array_angle]), axis=0)
            Mach_max_array = np.append(Mach_max_array, np.array([Mach_max_array_angle]), axis=0)
            MaxQ_array = np.append(MaxQ_array, np.array([MaxQ_array_angle]), axis=0)
            time_hard_landing_array = np.append(time_hard_landing_array, np.array([time_hard_landing_array_angle]), axis=0)
            hard_landing_points.append(hard_landing_points_angle)
            time_sepa2_array = np.append(time_sepa2_array, np.array([time_sepa2_array_angle]), axis=0)
            time_soft_landing_array = np.append(time_soft_landing_array, np.array([time_soft_landing_array_angle]), axis=0)
            soft_landing_points.append(soft_landing_points_angle)

        pd.DataFrame(time_apogee_array, index=self.index, columns=self.header).to_csv(self.result_dir + '/time_apogee.csv')
        pd.DataFrame(Vel_air_apogee_array, index=self.index, columns=self.header).to_csv(self.result_dir + '/vel_apogee.csv')
        pd.DataFrame(altitude_apogee_array, index=self.index, columns=self.header).to_csv(self.result_dir + '/altitude_apogee.csv')
        pd.DataFrame(downrange_hard_array, index=self.index, columns=self.header).to_csv(self.result_dir + '/downrange_hard.csv')
        pd.DataFrame(downrange_soft_array, index=self.index, columns=self.header).to_csv(self.result_dir + '/downrange_soft.csv')
        pd.DataFrame(Vel_air_max_array, index=self.index, columns=self.header).to_csv(self.result_dir + '/vel_max.csv')
        pd.DataFrame(Mach_max_array, index=self.index, columns=self.header).to_csv(self.result_dir + '/mach_max.csv')
        pd.DataFrame(MaxQ_array, index=self.index, columns=self.header).to_csv(self.result_dir + '/maxQ.csv')
        pd.DataFrame(time_hard_landing_array, index=self.index, columns=self.header).to_csv(self.result_dir + '/time_hard_landing.csv')
        pd.DataFrame(time_sepa2_array, index=self.index, columns=self.header).to_csv(self.result_dir + '/time_sepa_2nd.csv')
        pd.DataFrame(time_soft_landing_array, index=self.index, columns=self.header).to_csv(self.result_dir + '/time_soft_landing.csv')

        # self.single_solver.launch_site.wind_limit(self.Vel_wind_array, self.angle_wind_array - self.single_solver.launch_site.magnetic_declination(), hard_landing_points, soft_landing_points, self.result_dir)
        self.single_solver.launch_site.wind_limit(self.Vel_wind_array, self.angle_wind_array, hard_landing_points, soft_landing_points, self.result_dir)
        self.single_solver.launch_site.plot_kml(hard_landing_points, self.result_dir, name='hard')
        self.single_solver.launch_site.plot_kml(soft_landing_points, self.result_dir, name='soft')

        pd.DataFrame(soft_landing_points, index=self.index, columns=self.header).to_csv(self.result_dir + '/posENU_soft.csv')