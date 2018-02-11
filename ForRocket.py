# -*- coding: utf-8 -*-
import os
import sys
import json
import numpy as np
from scipy import interpolate
import matplotlib.pyplot as plt

# from BarrowmanFlow import BarrowmanFlow as bmf
from LaunchSiteData import LLHProvider as LaunchSiteProvider
from Simulator import rocket_dynamics as Simulator
# from FlightHeating import heating


def run(config_file):
    config = open(config_file)
    json_config = json.load(config)
    config.close()
    print('Config File: ', config_file)
    config = open(json_config.get('System').get('Engine Config json'))
    json_engine = json.load(config)
    config.close()
    print('Config File: ', json_config.get('System').get('Engine Config json'))

    # Make Result Directory
    model_name = json_config.get('System').get('Name')
    result_dir = json_config.get('System').get('Result Directory') + '_' + model_name
    if os.path.exists(result_dir):
        resultdir_org = result_dir
        i = 1
        while os.path.exists(result_dir):
            result_dir = resultdir_org + '_%02d' % (i)
            i = i + 1
    os.mkdir(result_dir)
    print ('Created Result Directory: ', result_dir)

    # Make Flight Object Instance
    rocket = Simulator.Rocket(json_config, json_engine)

    # Simulation
    single_condition = json_config.get('Wind').get('Single Condition Simulation')
    launch_site_name = json_config.get('Launch Pad').get('Site')
    launch_site = LaunchSiteProvider.LaunchSite(launch_site_name, result_dir)
    if single_condition:
        vel_wind = json_config.get('Wind').get('Wind Velocity [m/s]')
        angle_wind = json_config.get('Wind').get('Wind Direction [deg]')
        solver = Simulator.Solver(vel_wind, angle_wind, result_dir)
        solver.solve(rocket)
    else:
        wind = json_config.get('Wind')
        vel_wind_config = [wind.get('Wind Velocity Mini [m/s]'), wind.get('Wind Velocity Max [m/s]'), wind.get('Wind Velocity Step [m/s]')]
        angle_wind_config = [wind.get('Wind Direction Mini [deg]'), wind.get('Wind Direction Max [deg]'), wind.get('Wind Direction Step [deg]')]
        mappper = Simulator.Mapper4Wind(vel_wind_config, angle_wind_config, result_dir)
        vel_wind_array, angle_wind_array, hard_landing_points, soft_landing_points = mappper.mapping(rocket)
        launch_site.wind_limit(vel_wind_array, angle_wind_array, hard_landing_points, soft_landing_points)




if __name__ == '__main__':
    argv = sys.argv
    if len(argv) < 2:
        print ('Argument is missing')
        print ('Usage: python ForRocket.py configFileName.json')
        sys.exit()
    config_file = argv[1]
    run(config_file)

