# -*- coding: utf-8 -*-
import sys
import json
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# =====↓↓↓↓ USER INPUT ↓↓↓↓====
lug_clearance = 4.0  # [mm]
lug_clearance /= 1e3  # [mm => m]

# width_launcher = 70.0  # [mm]
# width_launcher /= 1e3  # [mm => m]
# fin_span = 150.0  # [mm]
# fin_span /= 1e3  # [mm => m]
# =====↑↑↑↑ USER INPUT ↑↑↑↑====


# config file arg
argv = sys.argv
if len(argv) < 3:
    print('Error!! argument is missing')
    print('Usage: python ForRocket.py configFileName.json ResultDirectory')
    sys.exit()   
if '.json' in argv[1]: 
    config_file = argv[1]
    result_dir = argv[2]
elif '.json' in argv[2]:
    config_file = argv[2]
    result_dir = argv[1]

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

# Pitch
elv0 = df['elevation'][0]
launcher_downrange = np.array([0.0, launcher_rail * np.cos(np.deg2rad(-elv0))])
launcher_altitude = np.array([0.0, launcher_rail * np.sin(np.deg2rad(-elv0))])
Pos_Lcg_downrange = np.sqrt(df['Pos_East'] ** 2 + df['Pos_North'] ** 2) + (R + lug_clearance) * np.sin(np.deg2rad(-df['elevation']))
Pos_Lcg_altitude = df['Pos_Up'] - (R + lug_clearance) * np.cos(np.deg2rad(-df['elevation']))
Pos_End_downrange = Pos_Lcg_downrange - (L - df['Lcg']) * np.cos(np.deg2rad(-df['elevation'])) - (R) * np.sin(np.deg2rad(-df['elevation']))
Pos_End_altitude = Pos_Lcg_altitude - (L - df['Lcg']) * np.sin(np.deg2rad(-df['elevation'])) + (R) * np.cos(np.deg2rad(-df['elevation']))

distance_End = np.sqrt(Pos_End_altitude ** 2 + Pos_End_downrange ** 2)
index_rocket_clear = np.array(distance_End > np.sqrt(launcher_altitude[-1] ** 2 + launcher_downrange[-1] ** 2)).argmax() + 1

plt.figure()
plt.plot(launcher_downrange, launcher_altitude, label='Launcher', color='black')
plt.plot(Pos_End_downrange[:index_rocket_clear], Pos_End_altitude[:index_rocket_clear], label='Trajectory - End')
plt.plot(Pos_Lcg_downrange[:index_rocket_clear], Pos_Lcg_altitude[:index_rocket_clear], label='Trajectory - Lcg')
plt.xlabel('Downrange [m]')
plt.ylabel('Altitude [m]')
plt.legend()
plt.grid()
plt.show()

distance = lug_clearance - ((L - df['Lcg']) * np.tan(np.deg2rad(df['elevation'] - elv0)))

plt.figure()
plt.plot(df['time'][:index_rocket_clear], distance[:index_rocket_clear] * 1e3)
plt.xlabel('Time [sec]')
plt.ylabel('Distance Launcher - Rocket end [mm]')
plt.axhline(y=0.0, color='black')
plt.xlim(xmin=0.0, xmax=df['time'][index_rocket_clear-1])
plt.grid()
plt.savefig(result_dir + '/Distance_launcher_rocket.png')

# Yaw
