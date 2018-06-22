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
lug_clearance /= 1e3  # [mm=>m]
# =====↑↑↑↑ USER INPUT ↑↑↑↑====

# config file to json
json = json.load(open(config_file))
struct = json.get('Structure')
L = struct.get('Length [m]')
d = struct.get('Diameter [m]')
R = 0.5 * d
lower_lug = struct.get('Lower Launch Lug FromNoseTip [m]')
launch_pad = json.get('Launch Pad')
launcher_rail = launch_pad.get('Launcher Rail Length [m]')

# read log
df = pd.read_csv(result_dir+'/log.csv', index_col=False)


time_launch_clear = float(open(result_dir+'/result.txt').readline().split(',')[1].split('[s]')[0])
index_launch_clear = np.argmax(df['time'] >= time_launch_clear)

# Pitch
elv0 = df['elevation'][0]
launcher_downrange = np.array([0.0, launcher_rail * np.cos(np.deg2rad(-elv0))])
launcher_altitude = np.array([0.0, launcher_rail * np.sin(np.deg2rad(-elv0))])
Pos_Lcg_downrange = np.sqrt(df['Pos_East'] ** 2 + df['Pos_North'] ** 2) + (R + lug_clearance) * np.sin(np.deg2rad(-df['elevation']))
Pos_Lcg_altitude = df['Pos_Up'] - (R + lug_clearance) * np.cos(np.deg2rad(-df['elevation']))
Pos_End_downrange = Pos_Lcg_downrange - (L - df['Lcg']) * np.cos(np.deg2rad(-df['elevation'])) - (R) * np.sin(np.deg2rad(-df['elevation']))
Pos_End_altitude = Pos_Lcg_altitude - (L - df['Lcg']) * np.sin(np.deg2rad(-df['elevation'])) + (R) * np.cos(np.deg2rad(-df['elevation']))


plt.figure()
plt.plot(launcher_downrange, launcher_altitude, label='Launcher', color='black')
plt.plot(Pos_End_downrange, Pos_End_altitude, label='Trajectory - End')
plt.plot(Pos_Lcg_downrange, Pos_Lcg_altitude, label='Trajectory - Lcg')
plt.legend()
plt.grid()
# plt.show()



distance = lug_clearance - ((L - df['Lcg']) * np.tan(np.deg2rad(df['elevation'] - elv0)))

plt.figure()
plt.plot(df['time'], distance * 1e3)
# plt.xlim(xmax=time_launch_clear)



plt.show()

# Yaw

# azimuth,elevation Lcg

