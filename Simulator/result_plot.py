import numpy as np
import pandas as pd
import datetime
import matplotlib.pyplot as plt
import simplekml
import pymap3d as pm
import pymap3d.vincenty

import Simulator.coordinate as coord


class Result:
    def __init__(self, result_dir):
        self.result_dir = result_dir

    ### Array List #######
    # time_onlauncher_log
    # pos_onlauncher_NED_log
    # vel_onlauncher_NED_log
    # omega_onlauncher_log
    # quat_onlauncher_log

    # pos_ECI_log
    # vel_ECI_log
    # omega_log
    # quat_log
    # attitude @ make_log
    # time_log
    # pos_NED_log @ make_log
    # pos_ECEF_log
    # pos_LLH_log
    # downrange_log @ make_log
    # vel_ECEF_log
    # vel_NED_log
    # vel_AIR_BODYframe_log
    # vel_AIR_BODYframe_abs_log @ make_log
    # alpha_log
    # beta_log
    # Ta_log
    # Pa_log
    # rho_log
    # Cs_log
    # Mach_log
    # dynamic_pressure_log
    # mdot_p_log
    # mdot_f_log
    # mdot_ox_log
    # thrust_log
    # Isp_log
    # mf_log
    # mox_log
    # mp_log
    # m_log
    # drag_log
    # normal_log
    # F_aero_WIND_log
    # F_aero_BODY_log
    # g_NED_log
    # force_BODY_log
    # acc_BODY_log
    # acc_ECI_log
    # Lcg_p_log
    # Lcg_log
    # Lcp_log
    # Ij_log
    # Ijdot_f_log
    # moment_aero_log
    # moment_aero_dumping_log
    # moment_jet_dumping_log
    # moment_log

    # time_decent_log
    # date_decent_log @ make_log
    # pos_decent_ECI_log
    # vel_decent_ECI_log
    # pos_decent_LLH_log @ make_log
    # downrange_decent_log @ make_log

    def __make_log(self, rocket):
        self.pos_onlauncer_LLH_log = np.array([pm.ned2geodetic(pos_NED[0], pos_NED[1], pos_NED[2], rocket.pos0_LLH[0], rocket.pos0_LLH[1], rocket.pos0_LLH[2]) for pos_NED in self.pos_onlauncher_NED_log])
        self.downrange_log = np.array([pm.vincenty.vdist(rocket.pos0_LLH[0], rocket.pos0_LLH[1], pos[0], pos[1]) for pos in self.pos_LLH_log])[:, 0]
        self.vel_AIR_BODYframe_abs_log = np.array([np.linalg.norm(vel) for vel in self.vel_AIR_BODYframe_log])
        DCM_NED2BODY_log = np.array([coord.DCM_NED2BODY_quat(quat) for quat in self.quat_log])
        self.attitude_log = np.array([coord.quat2euler(DCM) for DCM in DCM_NED2BODY_log])
        self.pos_NED_log = np.array([pm.ecef2ned(pos_ECEF[0], pos_ECEF[1], pos_ECEF[2], rocket.pos0_LLH[0], rocket.pos0_LLH[1], rocket.pos0_LLH[2]) for pos_ECEF in self.pos_ECEF_log])

        self.pos_decent_ECEF_log = np.array([coord.DCM_ECI2ECEF(t).dot(pos_ECI) for pos_ECI, t in zip(self.pos_decent_ECI_log, self.time_decent_log)])
        self.pos_decent_LLH_log = np.array([pm.ecef2geodetic(pos[0], pos[1], pos[2]) for pos in self.pos_decent_ECEF_log])
        self.vel_decent_NED_log = np.array([coord.DCM_ECEF2NED(rocket.pos0_LLH).dot(coord.vel_ECI2ECEF(vel_eci, coord.DCM_ECI2ECEF(t), pos_eci)) for vel_eci, t, pos_eci in zip(self.vel_decent_ECI_log, self.time_decent_log, self.pos_decent_ECI_log)])
        self.downrange_decent_log = np.array([pm.vincenty.vdist(rocket.pos0_LLH[0], rocket.pos0_LLH[1], pos[0], pos[1]) for pos in self.pos_decent_LLH_log])[:, 0]

        self.pos_hard_LLH_log = np.r_[self.pos_onlauncer_LLH_log, self.pos_LLH_log]
        index = np.argmax(self.time_log > self.time_decent_log[0])
        self.pos_soft_LLH_log = np.r_[self.pos_onlauncer_LLH_log, self.pos_LLH_log[:index, :], self.pos_decent_LLH_log]

    def __post_event(self, rocket):
        # Launch Clear
        self.time_launch_clear = self.time_onlauncher_log[-1]
        self.acc_gauge_launch_clear = rocket.thrust(self.time_launch_clear) / rocket.m(self.time_launch_clear)  # 本来はDragとかもあるがランチクリア時点では小さいので無視
        self.vel_launch_clear = coord.DCM_NED2BODY_quat(self.quat_onlauncher_log[-1, :]).dot(self.vel_onlauncher_NED_log[-1, :])[0]
        
        # Apogee
        index_apogee = np.argmax(self.pos_LLH_log[:, 2])
        self.time_apogee = self.time_log[index_apogee]
        self.alt_apogee = self.pos_LLH_log[index_apogee, 2]
        self.downrange_apogee = self.downrange_log[index_apogee]
        self.vel_apogee = self.vel_AIR_BODYframe_abs_log[index_apogee]

        # Max Q/Drag/Vel
        index = np.argmax(self.dynamic_pressure_log[:index_apogee])
        self.time_maxQ = self.time_log[index]
        self.alt_maxQ = self.pos_LLH_log[index, 2]
        self.maxQ = self.dynamic_pressure_log[index]
        index = np.argmax(self.vel_AIR_BODYframe_log[:index_apogee, 0])
        self.time_vel_max = self.time_log[index]
        self.alt_vel_max = self.pos_LLH_log[index, 2]
        self.vel_max = self.vel_AIR_BODYframe_log[index, 0]
        index = np.argmax(self.Mach_log[:index_apogee])
        self.time_Mach_max = self.time_log[index]
        self.alt_Mach_max = self.pos_LLH_log[index, 2]
        self.Mach_max = self.Mach_log[index]

        # Ballistic Landing
        self.time_hard_landing = self.time_log[-1]
        self.downrange_hard_landing = self.downrange_log[-1]
        self.vel_hard_landing = np.linalg.norm(coord.DCM_ECEF2NED(rocket.pos0_LLH).dot(coord.vel_ECI2ECEF(self.vel_ECI_log[-1, :], coord.DCM_ECI2ECEF(self.time_log[-1]), self.pos_ECI_log[-1, :])))

        # Decent Landing
        self.time_soft_landing = self.time_decent_log[-1]
        self.downrange_soft_landing = self.downrange_decent_log[-1]
        self.vel_soft_landing = coord.DCM_ECEF2NED(rocket.pos0_LLH).dot(coord.vel_ECI2ECEF(self.vel_decent_ECI_log[-1, :], coord.DCM_ECI2ECEF(self.time_decent_log[-1]), self.pos_decent_ECI_log[-1, :]))[2]

        # 2nd Separation
        if rocket.timer_mode:
            index_sepa2 = np.min([np.argmax(self.time_decent_log > rocket.t_2nd_max), np.max([np.argmax(self.pos_decent_LLH_log[:, 2] <= rocket.alt_sepa2), np.argmax(self.time_decent_log > rocket.t_2nd_min)])])
        else:
            index_sepa2 = np.argmax(self.pos_decent_LLH_log[:, 2] <= rocket.alt_sepa2)
        self.time_separation_2nd = self.time_decent_log[index_sepa2]
        self.vel_decent_1st = coord.DCM_ECEF2NED(rocket.pos0_LLH).dot(coord.vel_ECI2ECEF(self.vel_decent_ECI_log[index_sepa2, :], coord.DCM_ECI2ECEF(self.time_decent_log[index_sepa2]), self.pos_decent_ECI_log[index_sepa2, :]))[2]

        # 1st Separation
        self.time_separation_1st = self.time_decent_log[0]
        self.vel_separation_1st = coord.DCM_ECEF2NED(rocket.pos0_LLH).dot(coord.vel_ECI2ECEF(self.vel_decent_ECI_log[0, :], coord.DCM_ECI2ECEF(self.time_decent_log[0]), self.pos_decent_ECI_log[0, :]))[2]

        txt = open(self.result_dir + '/result.txt', mode='w')
        if rocket.input_mag_dec:
            txt.writelines(['Initial True Azimuth,', str(round(rocket.azimuth0 - rocket.mag_dec , 2)), '[deg]\n'])
            txt.writelines(['Initial Magnetic Azimuth,', str(round(rocket.azimuth0 , 2)), '[deg]\n'])
        else:
            txt.writelines(['Initial True Azimuth,', str(round(rocket.azimuth0 , 2)), '[deg]\n'])
        txt.writelines(['Launcher Clear X+,', str(round(self.time_launch_clear, 3)), '[s]\n'])
        txt.writelines(['Launcher Clear Acceleration,', str(round(self.acc_gauge_launch_clear / 9.80665, 3)), '[G]\n'])
        txt.writelines(['Launcher Clear Velocity,', str(round(self.vel_launch_clear, 3)), '[m/s]\n'])
        txt.writelines(['Max Q X+,', str(round(self.time_maxQ, 3)), '[s]\n'])
        txt.writelines(['Max Q Altitude,', str(round(self.alt_maxQ, 3)), '[m]\n'])
        txt.writelines(['Max Q,', str(round(self.maxQ / 1000.0, 3)), '[kPa]\n'])
        txt.writelines(['Max Speed X+,', str(round(self.time_vel_max,3)), '[s]\n'])
        txt.writelines(['Max Speed Altitude,', str(round(self.alt_vel_max, 3)), '[m]\n'])
        txt.writelines(['Max Speed,', str(round(self.vel_max, 3)), '[m/s]\n'])
        txt.writelines(['Max Mach Number X+,', str(round(self.time_Mach_max, 3)), '[s]\n'])
        txt.writelines(['Max Mach Number Altitude,', str(round(self.alt_Mach_max, 3)), '[m]\n'])
        txt.writelines(['Max Mach Number,', str(round(self.Mach_max, 3)), '[-]\n'])
        txt.writelines(['Apogee X+,', str(round(self.time_apogee, 3)), '[s]\n'])
        txt.writelines(['Apogee Altitude,', str(round(self.alt_apogee, 3)), '[m]\n'])
        txt.writelines(['Apogee Downrange,', str(round(self.downrange_apogee, 3)), '[m]\n'])
        txt.writelines(['Apogee Air Velocity,', str(round(self.vel_apogee, 3)), '[m/s]\n'])
        txt.writelines(['Hard Landing X+,', str(round(self.time_hard_landing, 3)), '[s]\n'])
        txt.writelines(['Hard Landing Downrange,', str(round(self.downrange_hard_landing, 3)), '[m]\n'])
        txt.writelines(['Hard Landing Velocity,', str(round(self.vel_hard_landing, 3)), '[m/s]\n'])
        txt.writelines(['Hard Landing Point,', str(self.pos_hard_LLH_log[-1, 0:2]), '\n'])
        txt.writelines(['1st Parachute Opening X+,', str(round(self.time_separation_1st, 3)), '[s]\n'])
        if rocket.para2_exist:
            txt.writelines(['2nd Parachute Opening X+,', str(round(self.time_separation_2nd, 3)), '[s]\n'])
            txt.writelines(['2nd Parachute Opening Velocity,', str(round(self.vel_decent_1st, 3)), '[m/s]\n'])
        txt.writelines(['Soft Landing X+,', str(round(self.time_soft_landing, 3)), '[s]\n'])
        txt.writelines(['Soft Landing Downrange,', str(round(self.downrange_soft_landing, 3)), '[m]\n'])
        txt.writelines(['Soft Landing Velocity,', str(round(self.vel_soft_landing, 3)), '[m/s]\n'])
        txt.writelines(['Soft Landing Point,', str(self.pos_soft_LLH_log[-1, 0:2]), '\n'])
        txt.close()

        df = pd.DataFrame({'lat': [self.pos_hard_LLH_log[-1,0], self.pos_soft_LLH_log[-1,0]],
                           'lon': [self.pos_hard_LLH_log[-1,1], self.pos_soft_LLH_log[-1,1]]},
                            index=['hard', 'soft'])
        df.to_csv(self.result_dir+'/landing_point.csv')

    def __post_kml(self):
        kml = simplekml.Kml(open=1)
        log_LLH = []
        for i in range(len(self.pos_hard_LLH_log[:,2])):
            if 0 == i % 10:
                log_LLH.append([self.pos_hard_LLH_log[i,1], self.pos_hard_LLH_log[i,0], self.pos_hard_LLH_log[i,2]])
        line = kml.newlinestring()
        line.style.linestyle.width = 5
        line.style.linestyle.color = simplekml.Color.red
        line.extrude = 1
        line.altitudemode = simplekml.AltitudeMode.absolute
        line.coords = log_LLH
        line.style.linestyle.colormode = simplekml.ColorMode.random
        kml.save(self.result_dir + '/hard_trajectory.kml')
        kml = simplekml.Kml(open=1)
        log_LLH = []
        for i in range(len(self.pos_soft_LLH_log[:,2])):
            if 0 == i % 10:
                log_LLH.append([self.pos_soft_LLH_log[i,1], self.pos_soft_LLH_log[i,0], self.pos_soft_LLH_log[i,2]])
        line = kml.newlinestring()
        line.style.linestyle.width = 5
        line.style.linestyle.color = simplekml.Color.red
        line.extrude = 1
        line.altitudemode = simplekml.AltitudeMode.absolute
        line.coords = log_LLH
        line.style.linestyle.colormode = simplekml.ColorMode.random
        kml.save(self.result_dir + '/soft_trajectory.kml')
    
    def __post_log(self):
        output_array = np.c_[
            self.time_log,
            self.Ta_log,
            self.Pa_log,
            self.rho_log,
            self.mdot_p_log,
            self.mdot_f_log,
            self.mdot_ox_log,
            self.mf_log,
            self.mox_log,
            self.mp_log,
            self.m_log,
            self.Lcg_p_log,
            self.Lcg_log,
            self.Lcp_log,
            self.Ij_log,
            self.moment_aero_log,
            self.moment_aero_dumping_log,
            self.moment_jet_dumping_log,
            self.moment_log,
            self.omega_log,
            self.quat_log,
            np.rad2deg(self.alpha_log),
            np.rad2deg(self.beta_log),
            self.attitude_log,
            self.thrust_log,
            self.Isp_log,
            self.drag_log,
            self.normal_log,
            self.F_aero_WIND_log,
            self.F_aero_BODY_log,
            self.g_NED_log,
            self.force_BODY_log,
            self.acc_BODY_log,
            self.acc_ECI_log,
            self.vel_AIR_BODYframe_log,
            self.vel_AIR_BODYframe_abs_log,
            self.Cs_log,
            self.Mach_log,
            self.dynamic_pressure_log,
            self.vel_ECI_log,
            self.vel_ECEF_log,
            self.vel_NED_log,
            self.pos_ECI_log,
            self.pos_ECEF_log,
            self.pos_LLH_log,
            self.downrange_log,
            self.pos_NED_log
        ]
        header = 'time,temp_air,press_air,rho_air,'\
                 'mdot_propellant,mdot_fuel,mdot_oxidizer,mass_fuel,mass_oxidizer,mass_propellant,mass,C.G._propellant,C.G.,C.P.,inertia_moment_x,inertia_moment_y,inertia_moment_z,'\
                 'moment_aero_x,moment_aero_y,moment_aero_z,moment_aero_dumping_x,moment_aero_dumping_y,moment_aero_dumping_z,moment_jet-dumping_x,moment_jet-dumping_y,moment_jet-dumping_z,'\
                 'moment_x,moment_y,moment_z,omega_x,omega_y,omega_z,quat0,quat1,quat2,quat3,alpha,beta,azimuth,elevation,roll,'\
                 'thrust,Isp,drag,normal,force_aero_wind_x,force_aero_wind_y,force_aero_wind_z,force_aero_body_x,force_aero_body_y,force_aero_body_z,'\
                 'gravity_NED_x,gravity_NED_y,gravity_NED_z,force_body_x,force_body_y,force_body_z,'\
                 'acc_body_x,acc_body_y,acc_body_z,acc_ECI_x,acc_ECI_y,acc_ECI_z,'\
                 'vel_air_body_x,vel_air_body_y,vel_air_body_z,vel_air_abs,SoundSpeed,Mach,dynamic_pressure,'\
                 'vel_ECI_x,vel_ECI_y,vel_ECI_z,vel_ECEF_x,vel_ECEF_y,vel_ECEF_z,vel_NED_x,vel_NED_y,vel_NED_z,'\
                 'pos_ECI_x,pos_ECI_y,pos_ECI_z,pos_ECEF_x,pos_ECEF_y,pos_ECEF_z,pos_LLH_x,pos_LLH_y,pos_LLH_z,downrange,pos_NED_x,pos_NED_y,pos_NED_z'
        np.savetxt(self.result_dir + '/log.csv', output_array, delimiter=',', header= header, comments='', fmt='%0.6f')

    def __post_graph(self, rocket):
        plt.close('all')

        plt.figure('Mass')
        plt.plot(self.time_log, self.mf_log, label='mass fuel')
        plt.plot(self.time_log, self.mox_log, label='mass oxidizer')
        plt.plot(self.time_log, self.m_log, label='mass all')
        plt.xlabel('Time [sec]')
        plt.ylabel('Mass [kg]')
        plt.xlim(left=0.0)
        plt.grid()
        plt.legend()
        plt.savefig(self.result_dir + '/Mass.png')

        plt.figure('C.G./C.P.')
        plt.plot(self.time_log, (self.Lcg_log / rocket.L) * 100.0, label='C.G.')
        plt.plot(self.time_log, (self.Lcp_log / rocket.L) * 100.0, label='C.P.')
        plt.xlabel('Time [sec]')
        plt.ylabel('From End [%]')
        plt.xlim(left=0.0)
        plt.grid()
        plt.legend()
        plt.savefig(self.result_dir + '/CG_CP.png')

        plt.figure('Anglar Velocity')
        plt.plot(self.time_log, self.omega_log[:, 0], label='BODY-X')
        plt.plot(self.time_log, self.omega_log[:, 1], label='BODY-Y')
        plt.plot(self.time_log, self.omega_log[:, 2], label='BODY-Z')
        plt.xlabel('Time [sec]')
        plt.ylabel('Anglar Velocity [rad/s]')
        plt.xlim(left=0.0)
        plt.grid()
        plt.legend()
        plt.savefig(self.result_dir + '/AnglarVelocity.png')

        plt.figure('Attitude')
        plt.plot(self.time_log, self.attitude_log[:, 0], label='azimuth')
        plt.plot(self.time_log, self.attitude_log[:, 1], label='elevation')
        plt.plot(self.time_log, self.attitude_log[:, 2], label='roll')
        plt.plot(self.time_log, self.alpha_log, label='alpha')
        plt.plot(self.time_log, self.beta_log, label='beta')
        plt.xlabel('Time [sec]')
        plt.ylabel('Attitude [deg]')
        plt.ylim(bottom=-180, top=180)
        plt.xlim(left=0.0)
        plt.grid()
        plt.legend()
        plt.savefig(self.result_dir + '/Attitude.png')

        plt.figure('Force')
        plt.plot(self.time_log, self.thrust_log, label='thrust')
        plt.plot(self.time_log, self.drag_log, label='drag')
        plt.xlabel('Time [sec]')
        plt.ylabel('Force [N]')
        plt.xlim(left=0.0)
        plt.grid()
        plt.legend()
        plt.savefig(self.result_dir + '/Force.png')

        plt.figure('Air Velocity')
        plt.plot(self.time_log, self.vel_AIR_BODYframe_log[:, 0], label='BODY-X')
        plt.plot(self.time_log, self.vel_AIR_BODYframe_log[:, 1], label='BODY-Y')
        plt.plot(self.time_log, self.vel_AIR_BODYframe_log[:, 2], label='BODY-Z')
        plt.xlabel('Time [sec]')
        plt.ylabel('Air Velocity [m/s]')
        plt.xlim(left=0.0)
        plt.grid()
        plt.legend()
        plt.savefig(self.result_dir + '/AirVelocity.png')

        plt.figure('Mach Number')
        plt.plot(self.time_log, self.Mach_log, label='Mach number')
        plt.xlabel('Time [sec]')
        plt.ylabel('Mach Number [-]')
        plt.xlim(left=0.0)
        plt.grid()
        plt.legend()
        plt.savefig(self.result_dir + '/Mach.png')

        plt.figure('Dynamic Pressure')
        plt.plot(self.time_log, self.dynamic_pressure_log / 1000.0, label='dynamic pressure')
        plt.xlabel('Time [sec]')
        plt.ylabel('Dynamic Pressure [kPa]')
        plt.xlim(left=0.0)
        plt.grid()
        plt.legend()
        plt.savefig(self.result_dir + '/Dynamicpressure.png')

        plt.figure('Body Acceleration')
        plt.plot(self.time_log, self.acc_BODY_log[:, 0], label='BODY-X')
        plt.plot(self.time_log, self.acc_BODY_log[:, 1], label='BODY-Y')
        plt.plot(self.time_log, self.acc_BODY_log[:, 2], label='BODY-Z')
        plt.xlabel('Time [sec]')
        plt.ylabel('Acceleration [m/s^2]')
        plt.xlim(left=0.0)
        plt.grid()
        plt.legend()
        plt.savefig(self.result_dir + '/Acceleration_BODY.png')

        plt.figure('ECI Acceleration')
        plt.plot(self.time_log, self.acc_ECI_log[:, 0], label='ECI-X')
        plt.plot(self.time_log, self.acc_ECI_log[:, 1], label='ECI-Y')
        plt.plot(self.time_log, self.acc_ECI_log[:, 2], label='ECI-Z')
        plt.xlabel('Time [sec]')
        plt.ylabel('Acceleration [m/s^2]')
        plt.xlim(left=0.0)
        plt.grid()
        plt.legend()
        plt.savefig(self.result_dir + '/Acceleration_ECI.png')

        plt.figure('ECI Velocity')
        plt.plot(self.time_log, self.vel_ECI_log[:, 0], label='ECI-X')
        plt.plot(self.time_log, self.vel_ECI_log[:, 1], label='ECI-Y')
        plt.plot(self.time_log, self.vel_ECI_log[:, 2], label='ECI-Z')
        plt.xlabel('Time [sec]')
        plt.ylabel('Velocity [m/s]')
        plt.xlim(left=0.0)
        plt.grid()
        plt.legend()
        plt.savefig(self.result_dir + '/Velocity_ECI.png')

        plt.figure('NED Velocity')
        plt.plot(self.time_log, self.vel_NED_log[:, 0], label='NED-X')
        plt.plot(self.time_log, self.vel_NED_log[:, 1], label='NED-Y')
        plt.plot(self.time_log, self.vel_NED_log[:, 2], label='NED-Z')
        plt.xlabel('Time [sec]')
        plt.ylabel('Velocity [m/s]')
        plt.xlim(left=0.0)
        plt.grid()
        plt.legend()
        plt.savefig(self.result_dir + '/Velocity_NED.png')

        plt.figure('ECEF Velocity')
        plt.plot(self.time_log, self.vel_ECEF_log[:, 0], label='ECEF-X')
        plt.plot(self.time_log, self.vel_ECEF_log[:, 1], label='ECEF-Y')
        plt.plot(self.time_log, self.vel_ECEF_log[:, 2], label='ECEF-Z')
        plt.xlabel('Time [sec]')
        plt.ylabel('Velocity [m/s]')
        plt.xlim(left=0.0)
        plt.grid()
        plt.legend()
        plt.savefig(self.result_dir + '/Velocity_ECEF.png')

        plt.figure('NED Position')
        plt.plot(self.time_log, self.pos_NED_log[:, 0], label='NED-X')
        plt.plot(self.time_log, self.pos_NED_log[:, 1], label='NED-Y')
        plt.plot(self.time_log, self.pos_NED_log[:, 2], label='NED-Z')
        plt.xlabel('Time [sec]')
        plt.ylabel('Position [m]')
        plt.xlim(left=0.0)
        plt.grid()
        plt.legend()
        plt.savefig(self.result_dir + '/Position_NED.png')

        plt.figure('ECI Position')
        plt.plot(self.time_log, self.pos_ECI_log[:, 0] / 1e3, label='ECI-X')
        plt.plot(self.time_log, self.pos_ECI_log[:, 1] / 1e3, label='ECI-Y')
        plt.plot(self.time_log, self.pos_ECI_log[:, 2] / 1e3, label='ECI-Z')
        plt.xlabel('Time [sec]')
        plt.ylabel('Position [km]')
        plt.xlim(left=0.0)
        plt.grid()
        plt.legend()
        plt.savefig(self.result_dir + '/Position_ECI.png')

        plt.figure('ECEF Position')
        plt.plot(self.time_log, self.pos_ECEF_log[:, 0] / 1e3, label='ECEF-X')
        plt.plot(self.time_log, self.pos_ECEF_log[:, 1] / 1e3, label='ECEF-Y')
        plt.plot(self.time_log, self.pos_ECEF_log[:, 2] / 1e3, label='ECEF-Z')
        plt.xlabel('Time [sec]')
        plt.ylabel('Position [km]')
        plt.xlim(left=0.0)
        plt.grid()
        plt.legend()
        plt.savefig(self.result_dir + '/Position_ECEF.png')

        plt.figure('Trajectory')
        plt.plot(self.downrange_log, self.pos_LLH_log[:,2])
        plt.xlabel('Downrange [m]')
        plt.ylabel('Altitude [m]')
        plt.xlim(left=0.0)
        plt.grid()
        plt.savefig(self.result_dir + '/Trajectory.png')

        plt.figure('Decent Trajectory')
        plt.plot(self.downrange_decent_log, self.pos_decent_LLH_log[:,2])
        plt.xlabel('Downrange [m]')
        plt.ylabel('Altitude [m]')
        plt.grid()
        plt.savefig(self.result_dir + '/DecentTrajectory.png')

        plt.figure('NED Decent Velocity')
        plt.plot(self.time_decent_log, self.vel_decent_NED_log[:, 0], label='NED-X')
        plt.plot(self.time_decent_log, self.vel_decent_NED_log[:, 1], label='NED-Y')
        plt.plot(self.time_decent_log, self.vel_decent_NED_log[:, 2], label='NED-Z')
        plt.xlabel('Time [sec]')
        plt.ylabel('Velocity [m/s]')
        plt.xlim(left=0.0)
        plt.grid()
        plt.legend()
        plt.savefig(self.result_dir + '/DecentVelocity_NED.png')

    def output_full(self, rocket):
        self.__make_log(rocket)
        self.__post_event(rocket)
        self.__post_log()
        self.__post_graph(rocket)
        self.__post_kml()
        if rocket.payload_exist:
            rocket.payload.result.output_full()
    
    def output(self, rocket):
        self.__make_log(rocket)
        self.__post_event(rocket)
        self.__post_log()
        if rocket.payload_exist:
            rocket.payload.result.output()


    def output_min(self, rocket):
        self.__make_log(rocket)
        self.__post_event(rocket)
        if rocket.payload_exist:
            rocket.payload.result.output()


class PayloadResult:
    def __init__(self, result_dir):
        self.result_dir = result_dir
        # pos_decent_LLH_log

    def __post_event(self):
        df = pd.DataFrame({'lat': [self.pos_decent_LLH_log[-1,0]],
                           'lon': [self.pos_decent_LLH_log[-1,1]]},
                            index=['soft'])
        df.to_csv(self.result_dir+'/payload_landing_point.csv')

    def __post_kml(self):
        kml = simplekml.Kml(open=1)
        log_LLH = []
        for i in range(len(self.pos_decent_LLH_log[:,2])):
            if 0 == i % 10:
                log_LLH.append([self.pos_decent_LLH_log[i,1], self.pos_decent_LLH_log[i,0], self.pos_decent_LLH_log[i,2]])
        line = kml.newlinestring()
        line.style.linestyle.width = 5
        line.style.linestyle.color = simplekml.Color.red
        line.extrude = 1
        line.altitudemode = simplekml.AltitudeMode.absolute
        line.coords = log_LLH
        line.style.linestyle.colormode = simplekml.ColorMode.random
        kml.save(self.result_dir + '/payload_trajectory.kml')

    def output_full(self):
        self.__post_event()
        self.__post_kml()
    
    def output(self):
        self.__post_event()