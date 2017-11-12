# -*- coding: utf-8 -*-
import os
import sys
import json
import numpy as np
from scipy import interpolate
import matplotlib.pyplot as plt

from Simulator import rocket_dynamics as Simulator
from BarrowmanFlow import BarrowmanFlow as bmf
from MapPlotter import MapPlotter as plotter
from FlightHeating import heating


class Rocket:
    def __init__(self, json):
        self.start_time = json.get('Start Time [sec]')
        self.end_time = json.get('End Time [sec]')
        self.timestep = json.get('Time Step [sec]')

        LP = json.get('Launch Pad')
        self.Pos0_LLH = LP.get('LLH [deg, deg, m]')
        self.azimuth0 = LP.get('Launch Azimuth [deg]')
        self.elevation0 = LP.get('Launch Elevation [deg]')
        if self.elevation0 < 0.0:
            self.elevation0 *= -1.0
        self.roll0 = LP.get('Launch Roll Angle [deg]')
        self.launcher_rail = LP.get('Launcher Rail Length [m]')

        struct = json.get('Structure')
        self.L = struct.get('Length [m]')
        self.d = struct.get('Diameter [m]')
        self.A = 0.25 * np.pi * self.d ** 2
        self.ms = struct.get('Structure Mass [kg]')
        self.Lcg_s = struct.get('Structure Length-C.G. FormNoseTip [m]')
        self.Lcg0_ox = struct.get('Initial Oxidizer Length-C.G. FromEnd [m]')  # ノーズからへの変換はメインルーチン内
        self.Lcg_f = self.L - struct.get('Fuel Length-C.G. FromEnd [m]')
        self.L_motor = struct.get('Length End-to-Tank [m]')
        self.Ij0_s_roll = struct.get('Structure Inertia-Moment Roll-Axis [kg*m^2]')
        self.Ij0_s_pitch = struct.get('Structure Inertia-Moment Pitch-Axis [kg*m^2]')

        prop = json.get('Propellant')
        self.m0_ox = prop.get('Oxidizer Mass [kg]')
        self.mdot_ox = prop.get('Oxidizer Mass Flow Rate [kg/s]')
        self.m0_f = prop.get('Fuel Mass Before Burn [kg]')
        self.mf_after = prop.get('Fuel Mass After Burn [kg]')
        self.mdot_f = prop.get('Fuel Mass Flow Rate [kg/s]')
        self.d_out_f = prop.get('Fuel Outside Diameter [m]')
        self.d_port_f = prop.get('Fuel Inside Diameter [m]')
        self.L_f = prop.get('Fuel Length [m]')

        aero = json.get('Aero')
        Cd_file_exist = aero.get('Cd File Exist')
        if Cd_file_exist:
            Cd_file = aero.get('Cd File')
            Cd_array = np.loadtxt(Cd_file, delimiter=',', skiprows=1)
            self.Cd = interpolate.interp1d(Cd_array[:,0], Cd_array[:,1], kind='cubic', bounds_error=False, fill_value=(Cd_array[0,1], Cd_array[-1,1]))
        else:
            Mach_array = np.arange(0.0, 20.01, 0.01)
            Cd_array = np.array([aero.get('Constant Cd')] * Mach_array.size)
            self.Cd = interpolate.interp1d(Mach_array, Cd_array, kind='linear', bounds_error=False, fill_value=(Cd_array[0], Cd_array[-1]))

        Lcp_file_exist = aero.get('Lcp File Exist')
        if Lcp_file_exist:
            Lcp_file = aero.get('Lcp File')
            Lcp_array = np.loadtxt(Lcp_file, delimiter=',', skiprows=1)
            self.Lcp = interpolate.interp1d(Lcp_array[:,0], Lcp_array[:,1], kind='cubic', bounds_error=False, fill_value=(Lcp_array[0,1], Lcp_array[-1,1]))
        else:
            Mach_array = np.arange(0.0, 20.01, 0.01)
            Lcp_array = np.array([aero.get('Constant Length-C.P. [m]')] * Mach_array.size)
            self.Lcp = interpolate.interp1d(Mach_array, Lcp_array, kind='linear', bounds_error=False, fill_value=(Lcp_array[0], Lcp_array[-1]))

        CNa_file_exist = aero.get('CNa File Exist')
        if CNa_file_exist:
            CNa_file = aero.get('CNa File')
            CNa_array = np.loadtxt(CNa_file, delimiter=',', skiprows=1)
            self.CNa = interpolate.interp1d(CNa_array[:,0], CNa_array[:,1], kind='cubic', bounds_error=False, fill_value=(CNa_array[0,1], CNa_array[-1,1]))
        else:
            Mach_array = np.arange(0.0, 20.01, 0.01)
            CNa_array = np.array([aero.get('Constant Normal Coefficient CNa')] * Mach_array.size)
            self.CNa = interpolate.interp1d(Mach_array, CNa_array, kind='linear', bounds_error=False, fill_value=(CNa_array[0], CNa_array[-1]))

        self.Clp = aero.get('Roll Dumping Moment Coefficient Clp')
        if self.Clp > 0.0:
            self.Clp *= -1.0
        self.Cmq = aero.get('Pitch Dumping Moment Coefficient Cmq')
        if self.Cmq > 0.0:
            self.Cmq *= -1.0
        self.Cnr = self.Cmq

        # Thrust, SL
        # 1st column : time [s]
        # 2nd column : thrust [N]
        engine = json.get('Engine')
        thrust_file_exist = engine.get('File Exist')
        if thrust_file_exist:
            thrust_file = engine.get('Thrust File')
            thrust_array = np.loadtxt(thrust_file, delimiter=',', skiprows=1)
            # 1G Cut
            time_array = thrust_array[:,0]
            thrust_array = thrust_array[:,1]
            index = (thrust_array > (self.ms + self.m0_f + self.m0_ox)).argmax()
            thrust_array = thrust_array[index:]
            time_array = time_array[index:] - time_array[index]
            self.thrust = interpolate.interp1d(time_array, thrust_array, kind='linear', bounds_error=False, fill_value=(0.0, 0.0))
        else:
            time_array = np.arange(0.0, engine.get('Burn Time [sec]') + 0.01, 0.01)
            thrust_array = np.array([engine.get('Constant Thrust [N]')] * time_array.size)
            self.thrust = interpolate.interp1d(time_array, thrust_array, kind='linear', bounds_error=False, fill_value=(0.0, 0.0))
        self.Isp = engine.get('Isp [sec]')
        self.dth = engine.get('Nozzle Throat Diameter [m]')
        self.eps = engine.get('Nozzle Expansion Ratio')
        self.Ath = 0.25 * np.pi * self.dth ** 2
        self.Ae = self.Ath * self.eps
        self.de = np.sqrt(self.Ae * 4.0 / np.pi)

        self.Wind_refAltitude = json.get('Environment').get('Wind Mesurement Height [m]')
        self.Wind_power_exp = json.get('Environment').get('Wind Power Law Exponential Coefficient')

        self.m0_p = self.m0_ox + self.m0_f
        self.m0 = self.ms + self.m0_p
        self.Lcg0_p = (self.m0_ox * (self.L - self.Lcg0_ox) + self.m0_f * self.Lcg_f) / self.m0_p
        self.Lcg0 = (self.ms * self.Lcg_s + self.m0_p * self.Lcg0_p) / self.m0


def run():
    argv = sys.argv
    if len(argv) < 2:
        print ('Argument is missing')
        print ('Usage: python ForRocket.py configFileName.json')
        sys.exit()
    config_file = argv[1]
    json_obj = json.load(open(config_file))
    print('Config File: ', config_file)

    model_name = json_obj.get('Name')
    resultdir = json_obj.get('System').get('Result Directory') + '_' + model_name
    if os.path.exists(resultdir):
        resultdir_org = resultdir
        i = 1
        while os.path.exists(resultdir):
            resultdir = resultdir_org + '_%02d' % (i)
            i = i + 1
    os.mkdir(resultdir)
    print ('Created Result Directory: ', resultdir)

    rocket = Rocket(json_obj)
    simulator = Simulator.Simulator(2.0, 0.0)
    time, ode_log = simulator.simulation(rocket)

    plt.figure()
    plt.plot(time, ode_log[:,0], label='X')
    plt.plot(time, ode_log[:,1], label='Y')
    plt.plot(time, ode_log[:,2], label='Z')
    plt.grid()
    plt.legend()

    plt.figure()
    plt.plot(time, ode_log[:,16], label='Mf')
    plt.plot(time, ode_log[:,17], label='Mox')    
    plt.grid()
    plt.legend()

    plt.show()


if __name__ == '__main__':
    run()

