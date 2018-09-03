# -*- coding: utf-8 -*-
import os
import sys
import re
import json
import numpy as np
import pandas as pd

from LaunchSiteData.Noshiro_land_3rd import NoshiroAsanai
from LaunchSiteData.Noshiro_sea import NoshiroOchiai
from LaunchSiteData.Taiki_land import TaikiLand

from Simulator.rocket_param import Rocket
from Simulator.solver import Solver
from Simulator.solver import WindMapSolver

# config file arg
argv = sys.argv
if len(argv) < 2:
    print('Error!! config file is missing')
    print('Usage: python ForRocket.py configFileName.json run_mode')
    print('-s, --single :    single trajectory')
    print('-m, --multi  :    map by multi trajectory for wind')
    sys.exit()   
config_file = argv[1]
try:
    run_mode = argv[2]
except IndexError:
    run_mode = '-s'


# config file to json
config = open(config_file)
json_config = json.load(config)
config.close()
print('Config File: ', config_file)
config = open(json_config.get('System').get('Engine Config json'))
json_engine = json.load(config)
config.close()
print('Config File: ', json_config.get('System').get('Engine Config json'))

# make launch site instance
launch_site_name = json_config.get('Launch Pad').get('Site')
noshiro = True if re.search(r'noshiro|nosiro', launch_site_name, re.IGNORECASE) else False
taiki = True if re.search(r'taiki', launch_site_name, re.IGNORECASE) else False
land = True if re.search(r'land', launch_site_name, re.IGNORECASE) else False
sea = True if re.search(r'sea', launch_site_name, re.IGNORECASE) else False
asanai = True if re.search(r'asanai|asauchi|asauti', launch_site_name, re.IGNORECASE) else False
field3rd = True if re.search(r'3rd', launch_site_name, re.IGNORECASE) else False
ochiai = True if re.search(r'ochiai|otiai', launch_site_name, re.IGNORECASE) else False

if noshiro and (land or asanai or field3rd):
    launch_site = NoshiroAsanai()
elif noshiro and (sea or oshiai):
    launch_site = NoshiroOchiai(3000.0)
elif taiki:
    launch_site = TaikiLand()
else:
    print('Error!! Not found launch site: ', launch_site_name)
    sys.exit()
print('Launch Site: ', launch_site.name)

# make rocket instance
model_name = json_config.get('System').get('Name')
rocket = Rocket(json_config, json_engine)
print('Model: ', model_name)


if run_mode == '1' or run_mode == '-s' or run_mode == '--single':
    # Make Result Directory
    result_dir = './Result_single_' + model_name
    if os.path.exists(result_dir):
        resultdir_org = result_dir
        i = 1
        while os.path.exists(result_dir):
            result_dir = resultdir_org + '_%02d' % (i)
            i = i + 1
    os.mkdir(result_dir)
    print('Created Result Directory: ', result_dir)

    # make solver instance
    solver = Solver(json_config, launch_site, result_dir)

    # run solver
    print('Running Solver...')
    solver.solve_dynamics(rocket)
    print('Completed solve!')
    print('Output Result Directory: ', result_dir)    

elif run_mode == '2' or run_mode == '-m' or run_mode == '--multi':
    # Make Result Directory
    result_dir = './Result_WindMap_' + model_name
    if os.path.exists(result_dir):
        resultdir_org = result_dir
        i = 1
        while os.path.exists(result_dir):
            result_dir = resultdir_org + '_%02d' % (i)
            i = i + 1
    os.mkdir(result_dir)
    print('Created Result Directory: ', result_dir)

    # make solver instance    
    vel_wind_config = [1.0, 7.0, 1.0]
    angle_wind_config = [0.0, 315.0, 45.0]
    single_solver = Solver(json_config, launch_site, '')
    solver = WindMapSolver(vel_wind_config, angle_wind_config, single_solver, result_dir)

    # run solver    
    print('Running Solver...')    
    solver.solve_map(rocket)
    print('Completed solve!')
    print('Output Result Directory: ', result_dir)

# elif run_mode == '3' or run_mode == 'wind_var':

else:
    print('Error!! run mode is undefined: ', run_mode)
    print('Usage:')
    print('-s, --single :    single trajectory')
    print('-m, --multi  :    map by multi trajectory for wind')
    sys.exit()