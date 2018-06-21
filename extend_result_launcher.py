# -*- coding: utf-8 -*-
import os
import sys
import json
import numpy as np
import pandas as pd
from scipy import interpolate
import matplotlib.pyplot as plt

from Simulator.rocket_param import Rocket


# =====↓↓↓↓ USER INPUT ↓↓↓↓====
config_file = 'rocket_config.json'
result_dir = './Result_single_A'

lug_clearance = 4.0  # [mm]
# =====↑↑↑↑ USER INPUT ↑↑↑↑====

# config file to json
json = json.load(open(config_file))
struct = json.get('Structure')
launch_pad = json.get('Launch Pad')

L = struct.get('Length [m]')
d = struct.get('Diameter [m]')

launcher_rail = launch_pad.get('Launcher Rail Length [m]')

df = pd.read_csv(result_dir+'/log.csv', index_col=False)

time_launch_clear = float(open(result_dir+'/result.txt').readline().split(',')[1].split('[s]')[0])
index_launch_clear = np.argmax(df['time'] >= time_launch_clear)

# Pitch
elv0 = df['elevation'][0]
distance = lug_clearance / 1e3 - ((L - df['Lcg']) * np.tan(np.deg2rad(df['elevation'] - elv0)))

plt.figure()
plt.plot(df['time'][:index_launch_clear], distance[:index_launch_clear] * 1e3)
plt.xlim(xmax=time_launch_clear)



plt.show()

# Yaw

# azimuth,elevationLcg

