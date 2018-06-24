# -*- coding: utf-8 -*-
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

import Simulator.heating as heating



# =====↓↓↓↓ USER INPUT ↓↓↓↓====
result_dir = './Result_single_A'

T_surface_init = 298.15  # Initial Surface Temperature [K]
R_nosetip = 0.0086  # Blunt Radius Tip [m]
thickness = 0.002  # Thickness Tip [m]
rho_nosetip = 1270.0  # Material Dencity [kg/m^3]
c = 1592.0  # Material Specific Heat [J/kg-K]
epsilon = 0.8  # Matrial Surface Emissivity
T_ablation = 600.0  # Ablation Temperature [K]
h_vaporization = 9288.48  # Vaporization Heat [kJ/kg]
# =====↑↑↑↑ USER INPUT ↑↑↑↑====

json = json.load(open(config_file))

heat_obj = heating.NoseCone()
heat_obj.T_surface_init = T_surface_init
heat_obj.R_nosetip = R_nosetip
heat_obj.thickness = thickness
heat_obj.rho = rho_nosetip
heat_obj.c = c
heat_obj.epsilon = epsilon
heat_obj.T_ablation = T_ablation
heat_obj.h_vaporization = h_vaporization / 1e3

# all_log = np.loadtxt(result_dir + '/log.csv', delimiter=',', skiprows=1)
df = pd.read_csv(result_dir+'/log.csv', index_col=False)

heater = heating.FlightHeating(df['time'], df['Vel_air_abs'], df['Pos_Up'])
heater.heating(heat_obj)
q_conv_log = heater.q_conv
q_rad_log = heater.q_rad
q_heat_log = q_conv_log + q_rad_log
T_surface_log = heater.T_surface
thickness_log = heater.thickness

plt.figure('Aerodynamics Heating')
plt.plot(df['time'], q_conv_log / 10**6, label='convection heat')
plt.plot(df['time'], q_rad_log / 10**6, label='radiation heat')
plt.plot(df['time'], q_heat_log / 10**6, label='total heat')
plt.xlabel('Time [sec]')
plt.ylabel('q dot [MW/m^2]')
plt.xlim(xmin=0.0)
plt.grid()
plt.legend()
plt.savefig(result_dir + '/AerodynamicsHeating.png')

plt.figure('Surface Temperature')
plt.plot(df['time'], T_surface_log, label='Surface Temperature')
plt.xlabel('Time [sec]')
plt.ylabel('Surface Temperature [K]')
plt.xlim(xmin=0.0)
plt.grid()
plt.legend()
plt.savefig(result_dir + '/SurfaceTemperature.png')