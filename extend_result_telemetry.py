import sys
import json
import re
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import pymap3d as pm
import simplekml

# =====↓↓↓↓ USER INPUT ↓↓↓↓====
receiving_point_LLH = np.array([40.248872, 140.012000, 0.0])
# =====↑↑↑↑ USER INPUT ↑↑↑↑====

# config file arg
argv = sys.argv
if len(argv) < 3:
    print('Error!! argument is missing')
    print('Usage: python extend_result_telemetry.py configFileName.json ResultDirectory')
    sys.exit()   
if '.json' in argv[1]: 
    config_file = argv[1]
    result_dir = argv[2]
elif '.json' in argv[2]:
    config_file = argv[2]
    result_dir = argv[1]

# config file to json
json = json.load(open(config_file))

# make launch site instance
pos_launch_LLH = json.get('Launch Pad').get('Site')
lat0 = pos_launch_LLH[0]
lon0 = pos_launch_LLH[1]
h0 = pos_launch_LLH[2]

df = pd.read_csv(result_dir+'/log.csv', index_col=False)
lat = df['pos_LLH_x']
lon = df['pos_LLH_y']
h = df['pos_LLH_z']
h.iat[-1] = 0.0

az, el, ran = pm.geodetic2aer(lat, lon, h, receiving_point_LLH[0], receiving_point_LLH[1], receiving_point_LLH[2])

df = df.assign(dist_Receive = ran)
df = df.assign(azi_Receive = az)
df = df.assign(elv_Receive = el)
df.to_csv(result_dir+'/log.csv', index=None)

plt.figure()
plt.plot(df['time'], ran)
plt.xlabel('Time [sec]')
plt.ylabel('Distance - 3D [m]')
plt.grid()
plt.savefig(result_dir + '/Distance_ReceivePoint.png')

plt.figure()
plt.plot(df['time'], el)
plt.xlabel('Time [sec]')
plt.ylabel('Elevation [deg]')
plt.grid()
plt.savefig(result_dir + '/Elevation_ReceivePoint.png')

plt.figure()
plt.plot(df['time'], az)
plt.xlabel('Time [sec]')
plt.ylabel('Azimuth [deg]')
plt.grid()
plt.savefig(result_dir + '/Azimuth_ReceivePoint.png')
