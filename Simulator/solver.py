import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy import interpolate
from scipy.integrate import odeint

import Simulator.coordinate as coord
import Simulator.environment as env
import Simulator.dynamics as dynamics


class Solver:
    def __init__(self, json, launch_site, result_dir):
        self.launch_site = launch_site
        self.result_dir = result_dir

        wind = json.get('Wind')
        launch_pad = json.get('Launch Pad')

        self.auto_end = json.get('Solver').get('Auto End Time')
        self.end_time = json.get('Solver').get('End Time [sec]')


        # Wind #########################################
        self.ref_alt = wind.get('Wind Mesurement Height [m]')
        self.wind_file_exist = wind.get('Wind File Exist')
        if self.wind_file_exist:
            wind_file = wind.get('Wind File')
            self.wind_array = np.loadtxt(wind_file, delimiter=',', skiprows=1)
            alt_array = self.wind_array[:, 0]  # [m]
            wind_speed_array = self.wind_array[:, 1]
            wind_direction_array = self.wind_array[:, 2]
        else:
            ref_speed = wind.get('Wind Velocity [m/s]')
            ref_direction = wind.get('Wind Direction [deg]')
            alt_array = np.arange(0.0, 20005.0, 5.0)
            wind_speed_array = self.launch_site.wind_law(alt_array, ref_speed, self.ref_alt)
            wind_direction_array = np.array([ref_direction] * alt_array.size)
        self.launch_site.wind_speed = interpolate.interp1d(alt_array, wind_speed_array, kind='linear', bounds_error=False, fill_value=(wind_speed_array[0], wind_speed_array[-1]))
        self.launch_site.wind_direction = interpolate.interp1d(alt_array, wind_direction_array + self.launch_site.magnetic_declination(), kind='linear', bounds_error=False, fill_value=(wind_direction_array[0], wind_direction_array[-1]))
        ################################################


        # Initial Attitude #############################
        self.azimuth0 = launch_pad.get('Launch Azimuth [deg]') + self.launch_site.magnetic_declination()
        self.elevation0 = launch_pad.get('Launch Elevation [deg]')
        if self.elevation0 > 0.0:
            self.elevation0 *= -1.0
        self.roll0 = launch_pad.get('Launch Roll Angle [deg]')
        self.launcher_rail = launch_pad.get('Launcher Rail Length [m]')
        ################################################


    def solve_dynamics(self, rocket):
        # initial postion
        length = (rocket.L - rocket.Lcg0) * np.cos(np.deg2rad(self.elevation0))
        X0_ENU = length * np.cos(np.deg2rad(self.azimuth0))
        Y0_ENU = length * np.sin(np.deg2rad(self.azimuth0))
        Z0_ENU = (rocket.L - rocket.Lcg0) * np.sin(np.deg2rad(np.abs(self.elevation0)))
        Pos0_ENU = np.array([X0_ENU, Y0_ENU, Z0_ENU])
        # initial attitude
        quat0 = coord.euler2quat(self.azimuth0, self.elevation0, self.roll0)

        # on Launcher ##################################################
        # lower launch lugがランチャレール長を越した時点でランチクリア
        # lower lugが抜けた時点での重心各値をメインのソルバへ渡す
        time = np.arange(0.0, 1.0 + np.sqrt(2.0 * self.launcher_rail * rocket.m(0.0) / rocket.ref_thrust), 0.001)

        if rocket.tipoff_exist:
            x0 = np.zeros(19)
            x0[0:3] = Pos0_ENU  # Pos_ENU
            x0[3:6] = np.zeros((1, 3))  # distance_Body
            x0[6:9] = np.zeros((1, 3))  # Vel_ENU
            x0[9:12] = np.zeros((1, 3))  # Vel_Body
            x0[12:15] = np.zeros((1, 3))  # omega_Body
            x0[15:19] = quat0  # quat
            ode_log = odeint(dynamics.onlauncher_tipoff_dynamics, x0, time, args=(rocket, self.launch_site, self.launcher_rail))
            omega_onLauncher_log = ode_log[:, 12:15]
            quat_onLauncher_log = ode_log[:, 15:19]
            quat_onLauncher_log = np.array(list(map(coord.quat_normalize, quat_onLauncher_log)))
        else:
            x0 = np.zeros(12)
            x0[0:3] = Pos0_ENU
            x0[3:6] = np.zeros((1, 3))
            x0[6:9] = np.zeros((1, 3))
            x0[9:12] = np.zeros((1, 3))
            ode_log = odeint(dynamics.onlauncher_dynamics, x0, time, args=(rocket, self.launch_site, quat0))
            omega_onLauncher_log = np.zeros((len(ode_log[:, 0]) , 3))
            quat_onLauncher_log = np.zeros((len(ode_log[:, 0]), 4)) + np.array([quat0] * len(ode_log[:, 0]))

        Pos_onLauncher_ENU_log = ode_log[:, 0:3]
        distance_Body_log = ode_log[:, 3:6]
        Vel_onLauncher_ENU_log = ode_log[:, 6:9]
        Vel_Body_log = ode_log[:, 9:12]
        ################################################################

        # on Launcher post #############################################
        # ランチクリアまででカット
        distance_lower_lug_log = (rocket.L - rocket.lower_lug) + distance_Body_log[:, 0]  # 下端ラグの機体後端からのオフセット
        index_launch_clear = np.argmax(distance_lower_lug_log >= self.launcher_rail)

        time_onLauncher_log = time[:index_launch_clear]
        Pos_onLauncher_ENU_log = Pos_onLauncher_ENU_log[:index_launch_clear, :]
        Vel_onLauncher_ENU_log = Vel_onLauncher_ENU_log[:index_launch_clear, :]
        omega_onLauncher_log = omega_onLauncher_log[:index_launch_clear, :]
        quat_onLauncher_log = quat_onLauncher_log[:index_launch_clear, :]

        DCM_ENU2Body_onLauncher_log = np.array(list(map(coord.DCM_ENU2Body_quat, quat_onLauncher_log)))
        attitude_onLauncher_log = np.array(list(map(coord.quat2euler, DCM_ENU2Body_onLauncher_log)))
        azimuth_onLauncher_log = attitude_onLauncher_log[:, 0]
        elevation_onLauncher_log = attitude_onLauncher_log[:, 1]
        roll_onLauncher_log = attitude_onLauncher_log[:, 2]

        # イベントでの値
        self.time_launch_clear = time_onLauncher_log[-1]
        self.altitude_launch_clear = Pos_onLauncher_ENU_log[-1, 2]
        self.vel_launch_clear = Vel_Body_log[index_launch_clear, 0]
        self.acc_launch_clear = rocket.thrust(self.time_launch_clear) / rocket.m(self.time_launch_clear)

        if rocket.tipoff_exist:
            moment_mass_tipoff = rocket.m(self.time_launch_clear) * env.gravity(self.altitude_launch_clear) * np.cos(np.deg2rad(elevation_onLauncher_log[-1])) * (rocket.lower_lug - rocket.Lcg(self.time_launch_clear))
            omegadot_mass_tipoff = moment_mass_tipoff / rocket.Ij_pitch(self.time_launch_clear)  # 下部ラグを支点とした質量でのチップオフ角加速度。あとで角速度に変換する
        else:
            omegadot_mass_tipoff = 0.0
        ################################################################

        # main flight ##################################################
        def estimate_end():
            It_digits = len(str(int(rocket.total_impulse)))
            return rocket.total_impulse / (10 ** (It_digits - 3))

        def dicide_timestep(end_time):
            time = int(end_time)
            digit = len(str(time))
            digit += round(int(str(time)[0])) // 10
            return 0.00001 * 10.0 ** digit

        start_time = self.time_launch_clear
        end_time = estimate_end() if self.auto_end else self.end_time
        time_step = dicide_timestep(end_time)
        time = np.arange(start_time, end_time, time_step)

        x0 = np.zeros(13)
        x0[0:3] = Pos_onLauncher_ENU_log[-1]
        x0[3:6] = Vel_onLauncher_ENU_log[-1]  # Vel_ENU
        x0[6:9] =  omega_onLauncher_log[-1] + np.array([0.0, omegadot_mass_tipoff * time_step, 0.0])  # omega_Body
        x0[9:13] = quat_onLauncher_log[-1]  # quat

        ode_log = odeint(dynamics.dynamics, x0, time, args=(rocket, self.launch_site))

        Pos_ENU_log = ode_log[:, 0:3]
        Vel_ENU_log = ode_log[:, 3:6]
        omega_log = ode_log[:, 6:9]
        quat_log = ode_log[:, 9:13]
        ################################################################

        # main flight post #############################################
        # 着地後の分をカット
        index_hard_landing = np.argmax(Pos_ENU_log[:, 2] <= 0.0)
        time_log = np.r_[time_onLauncher_log, time[:index_hard_landing+1]]
        Pos_ENU_log = np.r_[Pos_onLauncher_ENU_log, Pos_ENU_log[:index_hard_landing+1, :]]
        Vel_ENU_log = np.r_[Vel_onLauncher_ENU_log, Vel_ENU_log[:index_hard_landing+1, :]]
        omega_log = np.r_[omega_onLauncher_log, omega_log[:index_hard_landing+1, :]]
        quat_log = np.r_[quat_onLauncher_log, quat_log[:index_hard_landing+1, :]]

        quat_log = np.array(list(map(coord.quat_normalize, quat_log)))
        DCM_ENU2Body_log = np.array(list(map(coord.DCM_ENU2Body_quat, quat_log)))

        # odeint外をあとから計算
        mox_log = rocket.mox(time_log)
        mf_log = rocket.mf(time_log)
        mp_log = rocket.mp(time_log)
        m_log = rocket.m(time_log)
        thrust_SL_log = rocket.thrust(time_log)
        mdot_p_log = rocket.mdot_p(time_log)
        mdot_f_log = rocket.mdot_f(time_log)
        mdot_ox_log = rocket.mdot_ox(time_log)
        Lcg_ox_log = rocket.Lcg_ox(time_log)
        Lcg_p_log = rocket.Lcg_p(time_log)
        Lcg_log = rocket.Lcg(time_log)

        altitude_log = Pos_ENU_log[:, 2]
        downrange_log = np.array(list(map(np.linalg.norm, Pos_ENU_log[:, 0:2])))

        g_log = np.array(list(map(env.gravity, altitude_log)))
        P_air_log = env.get_std_press_array(altitude_log)
        rho_air_log = env.get_std_density_array(altitude_log)

        wind_ENU_log = np.array([env.Wind_ENU(self.launch_site.wind_speed(alt), self.launch_site.wind_direction(alt)) for alt in altitude_log])
        Vel_air_log = np.array([DCM.dot(vel - wind) for DCM, vel, wind in zip(DCM_ENU2Body_log, Vel_ENU_log, wind_ENU_log)])
        Vel_air_abs_log = np.array(list(map(np.linalg.norm, Vel_air_log)))
        alpha_log = np.rad2deg(np.arctan2(Vel_air_log[:, 2], Vel_air_log[:, 0]))
        beta_log = np.rad2deg(np.arcsin(-Vel_air_log[:, 1] / Vel_air_abs_log))
        Mach_log = Vel_air_abs_log / env.get_std_soundspeed_array(altitude_log)
        dynamic_pressure_log = 0.5 * rho_air_log * Vel_air_abs_log ** 2
        drag_log = dynamic_pressure_log * rocket.Cd(Mach_log) * rocket.A
        normal_log = dynamic_pressure_log * rocket.CNa(Mach_log) * rocket.A

        Lcp_log = rocket.Lcp(Mach_log)
        Fst_log = (Lcp_log - Lcg_log) / rocket.L * 100  # [%]

        attitude_log = np.array(list(map(coord.quat2euler, DCM_ENU2Body_log)))
        azimuth_log = attitude_log[:, 0]
        elevation_log = attitude_log[:, 1]
        roll_log = attitude_log[:, 2]

        pressure_thrust_log = (P_air_log[0] - P_air_log) * rocket.Ae
        thrust_log = np.array([thrust_SL + pressure_thrust if thrust_SL > 0.0 else 0.0 for thrust_SL, pressure_thrust in zip(thrust_SL_log, pressure_thrust_log)])
        Isp_log = rocket.Isp + np.array([pressure_thrust / (mdot_p * 9.80665) if mdot_p > 0.0 else 0.0 for pressure_thrust, mdot_p in zip(pressure_thrust_log, mdot_p_log)])

        Force_log = np.c_[thrust_log - drag_log, normal_log * np.deg2rad(beta_log), -normal_log * np.deg2rad(alpha_log)]
        Acc_Body_log = np.array([Force / m + DCM.dot(g) for Force, m, DCM, g in zip(Force_log, m_log, DCM_ENU2Body_log, np.c_[np.zeros_like(g_log), np.zeros_like(g_log), -g_log])])

        # イベントでの値
        # apogee
        if rocket.timer_mode:
            index_apogee = np.argmax(time_log > rocket.t_1st)
        else:
            index_apogee = np.argmax(altitude_log)

        self.time_apogee = time_log[index_apogee]
        self.altitude_apogee = altitude_log[index_apogee]
        self.Pos_ENU_apogee = Pos_ENU_log[index_apogee, :]
        self.Vel_ENU_apogee = Vel_ENU_log[index_apogee, :]
        self.Vel_air_abs_apogee = Vel_air_abs_log[index_apogee]
        self.downrange_apogee = np.linalg.norm(self.Pos_ENU_apogee[0:2])
        # Max Speed
        index_vel_max = np.argmax(Vel_air_abs_log[:index_apogee])
        self.time_vel_max = time_log[index_vel_max]
        self.altitude_vel_max = altitude_log[index_vel_max]
        self.Vel_air_max = Vel_air_abs_log[index_vel_max]

        # Max Mach Number
        index_Mach_max = np.argmax(Mach_log[:index_apogee])
        self.time_Mach_max = time_log[index_Mach_max]
        self.altitude_Mach_max = altitude_log[index_Mach_max]
        self.Mach_max = Mach_log[index_Mach_max]

        # Max Q
        index_maxQ = np.argmax(dynamic_pressure_log[:index_apogee])
        self.time_maxQ = time_log[index_maxQ]
        self.altitude_maxQ = altitude_log[index_maxQ]
        self.maxQ = dynamic_pressure_log[index_maxQ]

        # Hard Landing
        self.time_hard_landing = time_log[-1]
        self.hard_landing_point = Pos_ENU_log[-1, 0:2]
        self.downrange_hard_landing = np.linalg.norm(self.hard_landing_point)
        ################################################################


        #  パラシュート降下 ############################################
        x0 = np.zeros(4)
        x0[0:3] = self.Pos_ENU_apogee
        if self.Vel_ENU_apogee[2] > 0.0:
            x0[3] = 0.0
        else:
            x0[3] = self.Vel_ENU_apogee[2]
        est_landing = self.altitude_apogee / np.sqrt(2.0 * m_log[index_apogee] * env.gravity(0.0) / (env.get_std_density(0.0) * (rocket.CdS1 + rocket.CdS2)))
        time_decent_log = np.arange(self.time_apogee, 2*self.time_apogee+est_landing, 0.1)
        ode_log = odeint(dynamics.parachute_dynamics, x0, time_decent_log, args=(rocket, self.launch_site, m_log[index_apogee]))
        Pos_ENU_decent_log = ode_log[:, 0:3]
        Vel_descent_log = ode_log[:, 3]

        index_soft_landing = np.argmax(Pos_ENU_decent_log[:, 2] <= 0.0)
        if rocket.para2_exist:
            dV_decent = (Vel_descent_log[:-1] - Vel_descent_log[1:]) / (time_decent_log[1] - time_decent_log[0])
            index_sepa2 = np.argmin(dV_decent)
        else:
            index_sepa2 = index_soft_landing
        self.time_sepa2 = time_decent_log[index_sepa2]
        self.time_soft_landing = time_decent_log[index_soft_landing]
        self.soft_landing_point = Pos_ENU_decent_log[index_soft_landing, 0:2]
        self.downrange_soft_landing = np.linalg.norm(self.soft_landing_point)
        self.Vel_descent_1st = Vel_descent_log[index_sepa2-1]
        self.Vel_descent_2nd = Vel_descent_log[index_soft_landing]
        ################################################################

        # Result post ##################################################
        txt = open(self.result_dir + '/result.txt', mode='w')
        txt.writelines(['Launcher Clear X+,', str(round(self.time_launch_clear, 3)), '[s]\n'])
        txt.writelines(['Launcher Clear Acceleration,', str(round(self.acc_launch_clear / 9.80665, 3)), '[G]\n'])
        txt.writelines(['Launcher Clear Velocity,', str(round(self.vel_launch_clear, 3)), '[m/s]\n'])
        txt.writelines(['Max Q X+,', str(round(self.time_maxQ, 3)), '[s]\n'])
        txt.writelines(['Max Q Altitude,', str(round(self.altitude_maxQ, 3)), '[m]\n'])
        txt.writelines(['Max Q,', str(round(self.maxQ / 1000.0, 3)), '[kPa]\n'])
        txt.writelines(['Max Speed X+,', str(round(self.time_vel_max,3)), '[s]\n'])
        txt.writelines(['Max Speed Altitude,', str(round(self.altitude_vel_max, 3)), '[m]\n'])
        txt.writelines(['Max Speed,', str(round(self.Vel_air_max, 3)), '[m/s]\n'])
        txt.writelines(['Max Mach Number X+,', str(round(self.time_Mach_max, 3)), '[s]\n'])
        txt.writelines(['Max Mach Number Altitude,', str(round(self.altitude_Mach_max, 3)), '[m]\n'])
        txt.writelines(['Max Mach Number,', str(round(self.Mach_max, 3)), '[-]\n'])
        txt.writelines(['Apogee X+,', str(round(self.time_apogee, 3)), '[s]\n'])
        txt.writelines(['Apogee Altitude,', str(round(self.altitude_apogee, 3)), '[m]\n'])
        txt.writelines(['Apogee Downrange,', str(round(self.downrange_apogee, 3)), '[m]\n'])
        txt.writelines(['Apogee Air Velocity,', str(round(self.Vel_air_abs_apogee, 3)), '[m/s]\n'])
        txt.writelines(['Hard Landing X+,', str(round(self.time_hard_landing, 3)), '[s]\n'])
        txt.writelines(['Hard Landing Downrange,', str(round(self.downrange_hard_landing, 3)), '[m]\n'])
        txt.writelines(['1st Parachute Opening X+,', str(round(time_log[index_apogee], 3)), '[s]\n'])
        txt.writelines(['1st Parachute Descent Velocity,', str(round(self.Vel_descent_1st, 3)), '[m/s]\n'])
        txt.writelines(['2nd Parachute Opening X+,', str(round(self.time_sepa2, 3)), '[s]\n'])
        txt.writelines(['2nd Parachute Descent Velocity,', str(round(self.Vel_descent_2nd, 3)), '[m/s]\n'])
        txt.writelines(['Soft Landing X+,', str(round(self.time_soft_landing, 3)), '[s]\n'])
        txt.writelines(['Soft Landing Downrange,', str(round(self.downrange_soft_landing, 3)), '[m]\n'])
        txt.close()

        output_array = np.c_[
            time_log,
            g_log,
            P_air_log,
            rho_air_log,
            wind_ENU_log,
            mdot_f_log,
            mdot_ox_log,
            mdot_p_log,
            mf_log,
            mox_log,
            mp_log,
            m_log,
            Lcg_ox_log,
            Lcg_p_log,
            Lcg_log,
            Lcp_log,
            Fst_log,
            alpha_log,
            beta_log,
            omega_log,
            quat_log,
            azimuth_log,
            elevation_log,
            roll_log,
            thrust_log,
            Isp_log,
            drag_log,
            normal_log,
            Force_log,
            Vel_air_log,
            Vel_air_abs_log,
            Mach_log,
            dynamic_pressure_log,
            Acc_Body_log,
            Vel_ENU_log,
            Pos_ENU_log,
        ]
        header = 'time,g,P_air,rho_air,wind_East,wind_North,wind_Up,'\
                 'mdot_f,mdot_ox,mdot_p,mf,mox,mp,m,Lcg_ox,Lcg_p,Lcg,Lcp,Fst,'\
                 'alpha,beta,omega_roll,omega_pitch,omega_yaw,'\
                 'quat0,quat1,quat2,quat3,azimuth,elevation,roll,'\
                 'thrust,Isp,drag,normal,Force_X,Force_Y,Force_Z,'\
                 'Vel_air_X,Vel_air_Y,Vel_air_Z,Vel_air_abs,Mach,dynamic_pressure,'\
                 'Acc_Body_X,Acc_Body_Y,Acc_Body_Z,Vel_East,Vel_North,Vel_Up,Pos_East,Pos_North,Pos_Up'
        np.savetxt(self.result_dir + '/log.csv', output_array, delimiter=',', header= header, fmt='%0.6f', comments='')

        plt.close('all')

        plt.figure('Mass')
        plt.plot(time_log, mf_log, label='mass fuel')
        plt.plot(time_log, mox_log, label='mass oxidizer')
        plt.plot(time_log, m_log, label='mass all')
        plt.xlabel('Time [sec]')
        plt.ylabel('Mass [kg]')
        plt.xlim(xmin=0.0)
        plt.grid()
        plt.legend()
        plt.savefig(self.result_dir + '/Mass.png')

        plt.figure('C.G./C.P.')
        plt.plot(time_log, (Lcg_log / rocket.L) * 100.0, label='C.G.')
        plt.plot(time_log, (Lcp_log / rocket.L) * 100.0, label='C.P.')
        plt.xlabel('Time [sec]')
        plt.ylabel('From Nose [%]')
        plt.xlim(xmin=0.0)
        plt.grid()
        plt.legend()
        plt.savefig(self.result_dir + '/CG_CP.png')

        plt.figure('Attitude')
        plt.plot(time_log, azimuth_log, label='azimuth')
        plt.plot(time_log, elevation_log, label='elevation')
        plt.plot(time_log, roll_log, label='roll')
        plt.plot(time_log, alpha_log, label='alpha')
        plt.plot(time_log, beta_log, label='beta')
        plt.xlabel('Time [sec]')
        plt.ylabel('Attitude [deg]')
        plt.ylim(ymin=-180, ymax=180)
        plt.xlim(xmin=0.0)
        plt.grid()
        plt.legend()
        plt.savefig(self.result_dir + '/Attitude.png')

        plt.figure('Force')
        plt.plot(time_log, thrust_log, label='thrust')
        plt.plot(time_log, drag_log, label='drag')
        plt.plot(time_log, Force_log[:,1], label='normal')
        plt.xlabel('Time [sec]')
        plt.ylabel('Force [N]')
        plt.xlim(xmin=0.0)
        plt.grid()
        plt.legend()
        plt.savefig(self.result_dir + '/Force.png')

        plt.figure('Air Velocity')
        plt.plot(time_log, Vel_air_log[:, 0], label='V_air X')
        plt.plot(time_log, Vel_air_log[:, 1], label='V_air Y')
        plt.plot(time_log, Vel_air_log[:, 2], label='V_air Z')
        plt.xlabel('Time [sec]')
        plt.ylabel('Air Velocity [m/s]')
        plt.xlim(xmin=0.0)
        plt.grid()
        plt.legend()
        plt.savefig(self.result_dir + '/VelocityAir.png')

        plt.figure('Mach Number')
        plt.plot(time_log, Mach_log, label='Mach number')
        plt.xlabel('Time [sec]')
        plt.ylabel('Mach Number [-]')
        plt.xlim(xmin=0.0)
        plt.grid()
        plt.legend()
        plt.savefig(self.result_dir + '/Mach.png')

        plt.figure('Dynamic Pressure')
        plt.plot(time_log, dynamic_pressure_log / 1000.0, label='dynamic pressure')
        plt.xlabel('Time [sec]')
        plt.ylabel('Dynamic Pressure [kPa]')
        plt.xlim(xmin=0.0)
        plt.grid()
        plt.legend()
        plt.savefig(self.result_dir + '/Dynamicpressure.png')

        plt.figure('Body Acceleration')
        plt.plot(time_log, Acc_Body_log[:, 0], label='Acc_Body X')
        plt.plot(time_log, Acc_Body_log[:, 1], label='Acc_Body Y')
        plt.plot(time_log, Acc_Body_log[:, 2], label='Acc_Body Z')
        plt.xlabel('Time [sec]')
        plt.ylabel('Body Acceleration [m/s^2]')
        plt.xlim(xmin=0.0)
        plt.grid()
        plt.legend()
        plt.savefig(self.result_dir + '/AccelerationBody.png')

        plt.figure('ENU Velocity')
        plt.plot(time_log, Vel_ENU_log[:, 0], label='V_East')
        plt.plot(time_log, Vel_ENU_log[:, 1], label='V_North ')
        plt.plot(time_log, Vel_ENU_log[:, 2], label='V_Up ')
        plt.xlabel('Time [sec]')
        plt.ylabel('ENU Velocity [m/s]')
        plt.xlim(xmin=0.0)
        plt.grid()
        plt.legend()
        plt.savefig(self.result_dir + '/VelocityENU.png')

        plt.figure('ENU Position')
        plt.plot(time_log, Pos_ENU_log[:, 0], label='Pos_East')
        plt.plot(time_log, Pos_ENU_log[:, 1], label='Pos_North')
        plt.plot(time_log, Pos_ENU_log[:, 2], label='Pos_Up')
        plt.xlabel('Time [sec]')
        plt.ylabel('ENU Position [m]')
        plt.xlim(xmin=0.0)
        plt.grid()
        plt.legend()
        plt.savefig(self.result_dir + '/PositionENU.png')

        plt.figure('Trajectory')
        plt.plot(downrange_log / 1000.0, altitude_log / 1000.0)
        plt.xlabel('Downrange [km]')
        plt.ylabel('Altitude [km]')
        plt.xlim(xmin=0.0)
        plt.grid()
        plt.legend()
        plt.savefig(self.result_dir + '/Trajectory.png')

        plt.figure('Decent Velocity')
        plt.plot(time_decent_log[:index_soft_landing], Vel_descent_log[:index_soft_landing])
        plt.xlabel('Time [sec]')
        plt.ylabel('Velocity [m/s]')
        plt.xlim(xmin=self.time_apogee)
        plt.grid()
        plt.legend()
        plt.savefig(self.result_dir + '/DecentVel.png')

        plt.figure('Decent Altitude')
        plt.plot(time_decent_log[:index_soft_landing], Pos_ENU_decent_log[:index_soft_landing, 2])
        plt.xlabel('Time [sec]')
        plt.ylabel('Altitude [m]')
        plt.xlim(xmin=self.time_apogee)
        plt.grid()
        plt.legend()
        plt.savefig(self.result_dir + '/DecentAlt.png')

import os
import gc
from tqdm import tqdm

class WindMapSolver:
    def __init__(self, vel_wind_config, angle_wind_config, single_solver, result_dir):
        self.result_dir = result_dir
        self.single_solver = single_solver

        Vel_wind_min = vel_wind_config[0]
        Vel_wind_max = vel_wind_config[1]
        Vel_wind_step = vel_wind_config[2]
        angle_wind_min = angle_wind_config[0]
        angle_wind_max = angle_wind_config[1]
        angle_wind_step = angle_wind_config[2]

        self.Vel_wind_array = np.arange(Vel_wind_min, Vel_wind_max + Vel_wind_step, Vel_wind_step)
        self.angle_wind_array = np.arange(angle_wind_min, angle_wind_max + angle_wind_step, angle_wind_step)

        self.header = (self.angle_wind_array)
        self.index = (self.Vel_wind_array)

    def solve_map(self, rocket):
        time_apogee_array = np.empty((0, len(self.angle_wind_array)))
        Vel_air_apogee_array = np.empty((0, len(self.angle_wind_array)))
        altitude_apogee_array = np.empty((0, len(self.angle_wind_array)))
        downrange_hard_array = np.empty((0, len(self.angle_wind_array)))
        downrange_soft_array = np.empty((0, len(self.angle_wind_array)))
        Vel_air_max_array = np.empty((0, len(self.angle_wind_array)))
        Mach_max_array = np.empty((0, len(self.angle_wind_array)))
        MaxQ_array = np.empty((0, len(self.angle_wind_array)))
        time_hard_landing_array = np.empty((0, len(self.angle_wind_array)))
        hard_landing_points = []
        time_sepa2_array = np.empty((0, len(self.angle_wind_array)))
        time_soft_landing_array = np.empty((0, len(self.angle_wind_array)))
        soft_landing_points = []
        for Vel_wind in tqdm(self.Vel_wind_array):
            time_apogee_array_angle = np.empty(0)
            Vel_air_apogee_array_angle = np.empty(0)
            altitude_apogee_array_angle = np.empty(0)
            downrange_hard_array_angle = np.empty(0)
            downrange_soft_array_angle = np.empty(0)
            Vel_air_max_array_angle = np.empty(0)
            Mach_max_array_angle = np.empty(0)
            MaxQ_array_angle = np.empty(0)
            time_hard_landing_array_angle = np.empty(0)
            hard_landing_points_angle = []
            time_sepa2_array_angle = np.empty(0)
            time_soft_landing_array_angle = np.empty(0)
            soft_landing_points_angle = []
            for angle_wind in self.angle_wind_array:
                # run solver
                self.single_solver.result_dir = self.result_dir + '/Result_single_' + str(int(Vel_wind)) + str(int(angle_wind))
                os.mkdir(self.single_solver.result_dir)
                if self.single_solver.wind_file_exist:
                    alt_array = self.single_solver.wind_array[:, 0]
                else:
                    alt_array = np.arange(0.0, 20005.0, 5.0)
                    wind_speed_array = self.single_solver.launch_site.wind_law(alt_array, Vel_wind, self.single_solver.ref_alt)
                    self.single_solver.launch_site.wind_speed = interpolate.interp1d(alt_array, wind_speed_array, kind='linear', bounds_error=False, fill_value=(wind_speed_array[0], wind_speed_array[-1]))
                wind_direction_array = np.array([angle_wind] * alt_array.size)
                self.single_solver.launch_site.wind_direction = interpolate.interp1d(alt_array, wind_direction_array + self.single_solver.launch_site.magnetic_declination(), kind='linear', bounds_error=False, fill_value=(wind_direction_array[0], wind_direction_array[-1]))
                self.single_solver.solve_dynamics(rocket)

                # Result Pickup
                time_apogee_array_angle = np.append(time_apogee_array_angle, self.single_solver.time_apogee)
                Vel_air_apogee_array_angle = np.append(Vel_air_apogee_array_angle, self.single_solver.Vel_air_abs_apogee)
                altitude_apogee_array_angle = np.append(altitude_apogee_array_angle, self.single_solver.altitude_apogee)
                downrange_hard_array_angle = np.append(downrange_hard_array_angle, self.single_solver.downrange_hard_landing)
                downrange_soft_array_angle = np.append(downrange_soft_array_angle, self.single_solver.downrange_soft_landing)
                Vel_air_max_array_angle = np.append(Vel_air_max_array_angle, self.single_solver.Vel_air_max)
                Mach_max_array_angle = np.append(Mach_max_array_angle, self.single_solver.Mach_max)
                MaxQ_array_angle = np.append(MaxQ_array_angle, self.single_solver.maxQ)
                time_hard_landing_array_angle = np.append(time_hard_landing_array_angle, self.single_solver.time_hard_landing)
                hard_landing_points_angle.append(self.single_solver.hard_landing_point)
                time_sepa2_array_angle = np.append(time_sepa2_array_angle, self.single_solver.time_sepa2)
                time_soft_landing_array_angle = np.append(time_soft_landing_array_angle, self.single_solver.time_soft_landing)
                soft_landing_points_angle.append(self.single_solver.soft_landing_point)

                gc.collect()  # ガベージコレクションの開放、メモリクリアになってくれる？

            time_apogee_array = np.append(time_apogee_array, np.array([time_apogee_array_angle]), axis=0)
            Vel_air_apogee_array = np.append(Vel_air_apogee_array, np.array([Vel_air_apogee_array_angle]), axis=0)
            altitude_apogee_array = np.append(altitude_apogee_array, np.array([altitude_apogee_array_angle]), axis=0)
            downrange_hard_array = np.append(downrange_hard_array, np.array([downrange_hard_array_angle]), axis=0)
            downrange_soft_array = np.append(downrange_soft_array, np.array([downrange_soft_array_angle]), axis=0)
            Vel_air_max_array = np.append(Vel_air_max_array, np.array([Vel_air_max_array_angle]), axis=0)
            Mach_max_array = np.append(Mach_max_array, np.array([Mach_max_array_angle]), axis=0)
            MaxQ_array = np.append(MaxQ_array, np.array([MaxQ_array_angle]), axis=0)
            time_hard_landing_array = np.append(time_hard_landing_array, np.array([time_hard_landing_array_angle]), axis=0)
            hard_landing_points.append(hard_landing_points_angle)
            time_sepa2_array = np.append(time_sepa2_array, np.array([time_sepa2_array_angle]), axis=0)
            time_soft_landing_array = np.append(time_soft_landing_array, np.array([time_soft_landing_array_angle]), axis=0)
            soft_landing_points.append(soft_landing_points_angle)

        pd.DataFrame(time_apogee_array, index=self.index, columns=self.header).to_csv(self.result_dir + '/time_apogee.csv')
        pd.DataFrame(Vel_air_apogee_array, index=self.index, columns=self.header).to_csv(self.result_dir + '/vel_apogee.csv')
        pd.DataFrame(altitude_apogee_array, index=self.index, columns=self.header).to_csv(self.result_dir + '/altitude_apogee.csv')
        pd.DataFrame(downrange_hard_array, index=self.index, columns=self.header).to_csv(self.result_dir + '/downrange_hard.csv')
        pd.DataFrame(downrange_soft_array, index=self.index, columns=self.header).to_csv(self.result_dir + '/downrange_soft.csv')
        pd.DataFrame(Vel_air_max_array, index=self.index, columns=self.header).to_csv(self.result_dir + '/vel_max.csv')
        pd.DataFrame(Mach_max_array, index=self.index, columns=self.header).to_csv(self.result_dir + '/mach_max.csv')
        pd.DataFrame(MaxQ_array, index=self.index, columns=self.header).to_csv(self.result_dir + '/maxQ.csv')
        pd.DataFrame(time_hard_landing_array, index=self.index, columns=self.header).to_csv(self.result_dir + '/time_hard_landing.csv')
        pd.DataFrame(time_sepa2_array, index=self.index, columns=self.header).to_csv(self.result_dir + '/time_sepa_2nd.csv')
        pd.DataFrame(time_soft_landing_array, index=self.index, columns=self.header).to_csv(self.result_dir + '/time_soft_landing.csv')

        # self.single_solver.launch_site.wind_limit(self.Vel_wind_array, self.angle_wind_array - self.single_solver.launch_site.magnetic_declination(), hard_landing_points, soft_landing_points, self.result_dir)
        self.single_solver.launch_site.wind_limit(self.Vel_wind_array, self.angle_wind_array, hard_landing_points, soft_landing_points, self.result_dir)
        self.single_solver.launch_site.plot_kml(hard_landing_points, self.result_dir, name='hard')
        self.single_solver.launch_site.plot_kml(soft_landing_points, self.result_dir, name='soft')

        pd.DataFrame(soft_landing_points, index=self.index, columns=self.header).to_csv(self.result_dir + '/posENU_soft.csv')