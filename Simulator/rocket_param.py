import numpy as np
import pandas as pd
from scipy import interpolate
from scipy.integrate import odeint

import Simulator.heating as heating


class Rocket:
    def __init__(self, json, json_engine):
        struct = json.get('Structure')
        para = json.get('Parachute')
        aero = json.get('Aero')
        engine = json_engine.get('Engine')
        prop = json_engine.get('Propellant')


        # geometory ################################################
        self.L = struct.get('Length [m]')
        self.d = struct.get('Diameter [m]')
        self.A = 0.25 * np.pi * self.d ** 2

        self.dth = engine.get('Nozzle Throat Diameter [m]')
        self.eps = engine.get('Nozzle Expansion Ratio')
        self.Ath = 0.25 * np.pi * self.dth ** 2
        self.Ae = self.Ath * self.eps
        self.de = np.sqrt(self.Ae * 4.0 / np.pi)
        self.d_out_f = prop.get('Fuel Outside Diameter [m]')
        self.d_port_f = prop.get('Fuel Inside Diameter [m]')
        self.L_f = prop.get('Fuel Length [m]')

        self.tipoff_exist = struct.get('Tip-Off Calculation Exist')
        if self.tipoff_exist:
            self.upper_lug = struct.get('Upper Launch Lug FromNoseTip [m]')
            self.lower_lug = struct.get('Lower Launch Lug FromNoseTip [m]')
        else:
            self.lower_lug = self.L  # 機体後端に下端ラグがある仮定
        ############################################################


        # Mass parameter ###########################################
        self.ms = struct.get('Structure Mass [kg]')
        self.m0_f = prop.get('Fuel Mass Before Burn [kg]')
        self.m0_ox = prop.get('Oxidizer Mass [kg]')
        self.m0_p = self.m0_ox + self.m0_f
        self.m0 = self.ms + self.m0_p
        self.mf_after = prop.get('Fuel Mass After Burn [kg]')
        ############################################################


        # Thrust, SL ###############################################
        # 1st column : time [s]
        # 2nd column : thrust [N]
        thrust_file_exist = engine.get('File Exist')
        if thrust_file_exist:
            thrust_file = engine.get('Thrust File')
            thrust_array = np.loadtxt(thrust_file, delimiter=',', skiprows=1)
            # 1G Cut
            time_array = thrust_array[:,0]
            thrust_array = thrust_array[:,1]
            index = (thrust_array > (self.ms + self.m0_f + self.m0_ox) * 9.80665).argmax()
            thrust_array = thrust_array[index:]
            time_array = time_array[index:] - time_array[index]
            self.ref_thrust = np.max(thrust_array)
            self.thrust = interpolate.interp1d(time_array, thrust_array, kind='linear', bounds_error=False, fill_value=(0.0, 0.0))
            self.total_impulse = round(np.sum(thrust_array) * np.sum(time_array) / len(time_array))
        else:
            time_array = np.arange(0.0, engine.get('Burn Time [sec]') + 0.01, 0.01)
            thrust_array = np.array([engine.get('Constant Thrust [N]')] * time_array.size)
            self.ref_thrust = engine.get('Constant Thrust [N]')
            self.thrust = interpolate.interp1d(time_array, thrust_array, kind='linear', bounds_error=False, fill_value=(0.0, 0.0))
            self.total_impulse = round(engine.get('Constant Thrust [N]') * engine.get('Burn Time [sec]'))
        self.Isp = engine.get('Isp [sec]')
        ############################################################


        # Mass interpolate #########################################
        self.mdot_f = prop.get('Fuel Mass Flow Rate [kg/s]')        
        mdot_p_log = thrust_array / (self.Isp * 9.80665)
        self.mdot_p = interpolate.interp1d(time_array, mdot_p_log, kind='linear', bounds_error=False, fill_value=(0.0, 0.0))        
        mdot_ox_log = mdot_p_log - self.mdot_f
        self.mdot_ox = interpolate.interp1d(time_array, mdot_ox_log, kind='linear', bounds_error=False, fill_value=(0.0, 0.0))
        self.mdot_f = interpolate.interp1d(time_array, np.array([self.mdot_f]*len(time_array)), kind='linear', bounds_error=False, fill_value=(0.0, 0.0))

        mf_log_ode = odeint(lambda x, t: -self.mdot_f(t), self.m0_f, time_array)
        mf_log = mf_log_ode[0][0]
        for mf in mf_log_ode[1:]:
            mf_log = np.append(mf_log, mf[0])
        if (mf_log[-1] - self.mf_after) / self.mf_after > 0.05:
            print('Warning!! fuel mass at burn out is not matching')
        self.mf = interpolate.interp1d(time_array, mf_log, kind='linear', bounds_error=False, fill_value=(mf_log[0], mf_log[-1]))

        mox_log_ode = odeint(lambda x, t: -self.mdot_ox(t), self.m0_ox, time_array)
        mox_log = mox_log_ode[0][0]
        for mox in mox_log_ode[1:]:
            mox_log = np.append(mox_log, mox[0])
        self.mox = interpolate.interp1d(time_array, mox_log, kind='linear', bounds_error=False, fill_value=(mox_log[0], 0.0))

        mp_log = mf_log + mox_log
        self.mp = interpolate.interp1d(time_array, mp_log, kind='linear', bounds_error=False, fill_value=(mp_log[0], mp_log[-1]))

        m_log = self.ms + mp_log
        self.m = interpolate.interp1d(time_array, m_log, kind='linear', bounds_error=False, fill_value=(m_log[0], m_log[-1]))
        ############################################################


        # Center of Gravity parameter ##############################
        self.Lcg_s = struct.get('Structure Length-C.G. FormNoseTip [m]')
        self.Lcg0_ox = struct.get('Initial Oxidizer Length-C.G. FromEnd [m]')
        self.Lcg_f = self.L - struct.get('Fuel Length-C.G. FromEnd [m]')
        self.L_motor = struct.get('Length End-to-Tank [m]')
        self.Lcg0_p = (self.m0_ox * (self.L - self.Lcg0_ox) + self.m0_f * self.Lcg_f) / self.m0_p
        self.Lcg0 = (self.ms * self.Lcg_s + self.m0_p * self.Lcg0_p) / self.m0
        
        Lcg_ox_fromend = self.L_motor + (mox_log / self.m0_ox) * (self.Lcg0_ox - self.L_motor)
        Lcg_ox_log = self.L - Lcg_ox_fromend
        Lcg_p_log = (mf_log * self.Lcg_f + mox_log * Lcg_ox_log) / mp_log
        Lcg_log = (mp_log * Lcg_p_log + self.ms * self.Lcg_s) / m_log
        self.Lcg_ox = interpolate.interp1d(time_array, Lcg_ox_log, kind='linear', bounds_error=False, fill_value=(Lcg_ox_log[0], Lcg_ox_log[-1]))
        self.Lcg_p = interpolate.interp1d(time_array, Lcg_p_log, kind='linear', bounds_error=False, fill_value=(Lcg_p_log[0], Lcg_p_log[-1]))
        self.Lcg = interpolate.interp1d(time_array, Lcg_log, kind='linear', bounds_error=False, fill_value=(Lcg_log[0], Lcg_log[-1]))
        ############################################################


        # Inertia Moment parameter #################################
        self.Ij0_s_roll = struct.get('Structure Inertia-Moment Roll-Axis [kg*m^2]')
        self.Ij0_s_pitch = struct.get('Structure Inertia-Moment Pitch-Axis [kg*m^2]')
        Ij_f_pitch_log = mf_log * ((self.d_port_f ** 2 + self.d_out_f ** 2) / 4.0 + self.L_f / 3.0) / 4.0
        Ij_f_roll_log = mf_log * (self.d_port_f ** 2 + self.d_out_f ** 2) / 8.0
        # Offset
        Ij_s_pitch_log = self.Ij0_s_pitch + self.ms * np.abs(Lcg_log - self.Lcg_s) ** 2
        Ij_f_pitch_log += mf_log * np.abs(Lcg_log - self.Lcg_f) ** 2
        Ij_pitch_log = Ij_s_pitch_log + Ij_f_pitch_log
        Ij_roll_log = self.Ij0_s_roll + Ij_f_roll_log
        self.Ij_pitch = interpolate.interp1d(time_array, Ij_pitch_log, kind='linear', bounds_error=False, fill_value=(Ij_pitch_log[0], Ij_pitch_log[-1]))
        self.Ij_roll = interpolate.interp1d(time_array, Ij_roll_log, kind='linear', bounds_error=False, fill_value=(Ij_roll_log[0], Ij_roll_log[-1]))
        Ijdot_f_pitch_log = Ij_f_pitch_log * self.mdot_f(time_array) / mf_log
        Ijdot_f_roll_log = Ij_f_roll_log * self.mdot_f(time_array) / mf_log
        self.Ijdot_f_pitch = interpolate.interp1d(time_array, Ijdot_f_pitch_log, kind='linear', bounds_error=False, fill_value=(Ijdot_f_pitch_log[0], Ijdot_f_pitch_log[-1]))
        self.Ijdot_f_roll = interpolate.interp1d(time_array, Ijdot_f_roll_log, kind='linear', bounds_error=False, fill_value=(Ijdot_f_roll_log[0], Ijdot_f_roll_log[-1]))
        ############################################################


        # Aero parameter ###########################################
        Cd_file_exist = aero.get('Cd File Exist')
        if Cd_file_exist:
            Cd_file = aero.get('Cd File')
            Cd_array = np.loadtxt(Cd_file, delimiter=',', skiprows=1)
            self.Cd = interpolate.interp1d(Cd_array[:,0], Cd_array[:,1], kind='linear', bounds_error=False, fill_value=(Cd_array[0,1], Cd_array[-1,1]))
        else:
            Mach_array = np.arange(0.0, 20.01, 0.01)
            Cd_array = np.array([aero.get('Constant Cd')] * Mach_array.size)
            self.Cd = interpolate.interp1d(Mach_array, Cd_array, kind='linear', bounds_error=False, fill_value=(Cd_array[0], Cd_array[-1]))

        Lcp_file_exist = aero.get('Lcp File Exist')
        if Lcp_file_exist:
            Lcp_file = aero.get('Lcp File')
            Lcp_array = np.loadtxt(Lcp_file, delimiter=',', skiprows=1)
            self.Lcp = interpolate.interp1d(Lcp_array[:,0], Lcp_array[:,1], kind='linear', bounds_error=False, fill_value=(Lcp_array[0,1], Lcp_array[-1,1]))
        else:
            Mach_array = np.arange(0.0, 20.01, 0.01)
            Lcp_array = np.array([aero.get('Constant Length-C.P. [m]')] * Mach_array.size)
            self.Lcp = interpolate.interp1d(Mach_array, Lcp_array, kind='linear', bounds_error=False, fill_value=(Lcp_array[0], Lcp_array[-1]))

        CNa_file_exist = aero.get('CNa File Exist')
        if CNa_file_exist:
            CNa_file = aero.get('CNa File')
            CNa_array = np.loadtxt(CNa_file, delimiter=',', skiprows=1)
            self.CNa = interpolate.interp1d(CNa_array[:,0], CNa_array[:,1], kind='linear', bounds_error=False, fill_value=(CNa_array[0,1], CNa_array[-1,1]))
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
        ############################################################
        

        # Parachute parameter ######################################
        self.timer_mode = para.get('Timer Mode')
        self.CdS1 = para.get('1st Parachute CdS [m2]')
        self.t_1st = para.get('1st Timer [s]')
        self.para2_exist = para.get('2nd Parachute Exist')
        if self.para2_exist:
            self.CdS2 = para.get('2nd Parachute CdS [m2]')
            self.alt_sepa2 = para.get('2nd Parachute Opening Altitude [m]')
            self.t_2nd_min = para.get('2nd Timer Min [s]')
            self.t_2nd_max = para.get('2nd Timer Max [s]')
        else:
            self.CdS2 = 0.0
            self.alt_sepa2 = 0.0
            self.t_2nd_min = 0.0
            self.t_2nd_max = 0.0
        ############################################################

        