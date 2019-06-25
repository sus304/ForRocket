import datetime
import numpy as np
from scipy.integrate import odeint
import pymap3d as pm

import Simulator.coordinate as coord
import Simulator.environment as env
from Simulator.dynamics import dynamics_odeint, dynamics_result
from Simulator.launcher_dynamics import onlauncher_dynamics
from Simulator.parachute_dynamics import parachute_dynamics
from Simulator.parachute_dynamics import payload_parachute_dynamics


def solve_trajectory(rocket):
    # Initial Attitude #############################
    quat0 = coord.euler2quat(rocket.azimuth0, rocket.elevation0)
    DCM_LAUNCHER2NED = coord.DCM_NED2BODY_quat(quat0).transpose()
    ################################################
    
    # Initial Position #############################
    pos0_ECEF = pm.geodetic2ecef(rocket.pos0_LLH[0], rocket.pos0_LLH[1], rocket.pos0_LLH[2])
    pos0_ECI = pm.ecef2eci(pos0_ECEF, rocket.launch_date)
    pos0_NED = DCM_LAUNCHER2NED.dot(np.array([rocket.Lcg0, 0.0, 0.0]))
    ################################################

    # onLauncher Trajectory ##################################################
    # lower launch lugがランチャレール長を越した時点でランチクリア
    # lower lugが抜けた時点での重心各値をメインのソルバへ渡す
    time = np.arange(0.0, 0.5 + np.sqrt(2.0 * rocket.launcher_rail * rocket.m(0.0) / rocket.ref_thrust), 0.01)
    x0 = np.zeros(13)
    x0[0:3] = pos0_NED  # Pos_NED
    x0[3:6] = np.zeros((1, 3))  # Vel_NED
    x0[6:9] = np.zeros((1, 3))  # omega
    x0[9:13] = quat0  # quat
    ode_log = odeint(onlauncher_dynamics, x0, time, args=(rocket, DCM_LAUNCHER2NED.transpose()))

    # onLauncher post ######
    pos_LAUNCHER = np.array([DCM_LAUNCHER2NED.transpose().dot(pos) for pos in ode_log[:, 0:3]])
    # ランチクリアまででカット
    index_launch_clear = np.argmax(rocket.launcher_rail <= pos_LAUNCHER[:, 0] - rocket.lower_lug)
    # log
    rocket.result.time_onlauncher_log = time[:index_launch_clear+1]
    rocket.result.pos_onlauncher_NED_log = ode_log[:index_launch_clear+1, 0:3]
    rocket.result.vel_onlauncher_NED_log = ode_log[:index_launch_clear+1, 3:6]
    rocket.result.omega_onlauncher_log = ode_log[:index_launch_clear+1, 6:9]
    rocket.result.quat_onlauncher_log = ode_log[:index_launch_clear+1, 9:13]
    
    pos_launch_clear_NED = rocket.result.pos_onlauncher_NED_log[-1, :]
    vel_launch_clear_NED = rocket.result.vel_onlauncher_NED_log[-1, :]
    omega_launch_clear = rocket.result.omega_onlauncher_log[-1, :]
    quat_launch_clear = rocket.result.quat_onlauncher_log[-1, :]
    time_launch_clear = rocket.result.time_onlauncher_log[-1]
    altitude_launch_clear = -pos_launch_clear_NED[2]
    DCM_NED2BODY = coord.DCM_NED2BODY_quat(quat_launch_clear)

    # tipoff moment
    if rocket.tipoff_exist:
        mg_tipoff_NED = np.array([0, 0, rocket.m(time_launch_clear) * env.gravity(altitude_launch_clear)])
        mg_tipoff_BODY = DCM_NED2BODY.dot(mg_tipoff_NED)
        moment_arm_tipoff = np.array([rocket.lower_lug - rocket.Lcg(time_launch_clear), 0, 0])
        moment_tipoff = np.cross(mg_tipoff_BODY, moment_arm_tipoff)
        Ij_pitch = rocket.Ij_pitch(time_launch_clear) + rocket.m(time_launch_clear) * np.abs(rocket.Lcg(time_launch_clear) - rocket.lower_lug) ** 2
        Ij_roll = rocket.Ij_roll(time_launch_clear)
        Ij = np.array([Ij_roll, Ij_pitch, Ij_pitch])
        omegadot_tipoff = moment_tipoff / Ij  # 下部ラグを支点とした機体質量でのチップオフ角加速度。あとで角速度に変換する
    else:
        omegadot_tipoff = 0.0
    ################################################

    # Main Trajectory ##################################################
    def estimate_end(total_impulse):
        It = total_impulse
        tf = 2.37 * It ** 0.37
        tf_order = len(str(int(tf))) - 1
        dt = 10.0 ** (tf_order - 4)
        return 1.4*tf, dt
    if rocket.auto_end:
        rocket.end_time, rocket.time_step = estimate_end(rocket.total_impulse)
    start_time = time_launch_clear# + rocket.time_step
    time = np.arange(start_time, rocket.end_time, rocket.time_step)

    pos_launch_clear_ECEF = pm.ned2ecef(pos_launch_clear_NED[0], pos_launch_clear_NED[1], pos_launch_clear_NED[2], rocket.pos0_LLH[0], rocket.pos0_LLH[1], rocket.pos0_LLH[2])
    pos_launch_clear_ECI = coord.DCM_ECI2ECEF(time_launch_clear).transpose().dot(pos_launch_clear_ECEF)
    DCM_NED2ECEF = coord.DCM_ECEF2NED(rocket.pos0_LLH).transpose()
    DCM_ECI2ECEF = coord.DCM_ECI2ECEF(time_launch_clear)

    x0 = np.zeros(13)
    x0[0:3] = pos_launch_clear_ECI
    x0[3:6] = coord.vel_ECEF2ECI(DCM_NED2ECEF.dot(vel_launch_clear_NED), DCM_ECI2ECEF, pos_launch_clear_ECI)
    x0[6:9] = omega_launch_clear + omegadot_tipoff * rocket.time_step
    x0[9:13] = quat_launch_clear
    ode_log = odeint(dynamics_odeint, x0, time, args=(rocket, ))

    # Main post ######
    date_array = rocket.launch_date + np.array([datetime.timedelta(seconds=sec) for sec in time])
    pos_ECEF = np.array([coord.DCM_ECI2ECEF(t).dot(pos) for pos, t in zip(ode_log[:, 0:3], time)])
    pos_LLH = np.array([pm.ecef2geodetic(pos[0], pos[1], pos[2]) for pos in pos_ECEF])

    # 着地後の分をカット
    index_hard_landing = np.argmax(pos_LLH[:, 2] <= 0.0)
    # log
    rocket.result.time_log = time[:index_hard_landing+1]
    rocket.result.pos_ECI_log = ode_log[:index_hard_landing+1, 0:3]
    rocket.result.vel_ECI_log = ode_log[:index_hard_landing+1, 3:6]
    rocket.result.omega_log = ode_log[:index_hard_landing+1, 6:9]
    rocket.result.quat_log = ode_log[:index_hard_landing+1, 9:13]
    dynamics_result(rocket)
    ################################################
    
    # Decent Trajectory ##################################################
    if rocket.timer_mode:
        index_apogee = np.argmax(rocket.result.time_log > rocket.t_1st)
    else:
        index_apogee = np.argmax(pos_LLH[:, 2])
    time_apogee = rocket.result.time_log[index_apogee]
    pos_apogee_ECI = rocket.result.pos_ECI_log[index_apogee]
    vel_apogee_ECI = rocket.result.vel_ECI_log[index_apogee]
    vel_apogee_ned = np.array([0, 0, coord.DCM_ECEF2NED(rocket.pos0_LLH).dot(coord.vel_ECI2ECEF(vel_apogee_ECI, coord.DCM_ECI2ECEF(time_apogee), pos_apogee_ECI))[2]])
    vel_apogee_ECI = coord.vel_ECEF2ECI(coord.DCM_ECEF2NED(rocket.pos0_LLH).transpose().dot(vel_apogee_ned), coord.DCM_ECI2ECEF(time_apogee), pos_apogee_ECI)

    if rocket.payload_exist:
        rocket.m_burnout -= rocket.payload.mass
        est_landing_payload = pos_LLH[index_apogee, 2] / np.sqrt(2.0 * rocket.payload.mass * env.gravity(pos_LLH[index_apogee, 2]) / (env.get_std_density(0.0) * rocket.payload.CdS))
    est_landing = pos_LLH[index_apogee, 2] / np.sqrt(2.0 * rocket.result.m_log[index_apogee] * env.gravity(pos_LLH[index_apogee, 2]) / (env.get_std_density(0.0) * (rocket.CdS1 + rocket.CdS2)))
    time = np.arange(time_apogee, time_apogee + 1.2 * est_landing, 0.1)
    x0 = np.zeros(6)
    x0[0:3] = pos_apogee_ECI
    x0[3:6] = vel_apogee_ECI
    ode_log = odeint(parachute_dynamics, x0, time, args=(rocket, ))

    # Decent post ######
    date_array = rocket.launch_date + np.array([datetime.timedelta(seconds=sec) for sec in time])
    pos_ECEF = np.array([coord.DCM_ECI2ECEF(t).dot(pos) for pos, t in zip(ode_log[:, 0:3], time)])
    pos_LLH = np.array([pm.ecef2geodetic(pos[0], pos[1], pos[2]) for pos in pos_ECEF])

    # 着地後の分をカット
    index_soft_landing = np.argmax(pos_LLH[:, 2] <= 0.0)
    # log
    rocket.result.time_decent_log = time[:index_soft_landing+1]
    rocket.result.pos_decent_ECI_log = ode_log[:index_soft_landing+1, 0:3]
    rocket.result.vel_decent_ECI_log = ode_log[:index_soft_landing+1, 3:6]

    # Payload ####
    if rocket.payload_exist:
        time = np.arange(time_apogee, time_apogee + 1.2 * est_landing_payload, 0.01)
        x0 = np.zeros(6)
        x0[0:3] = pos_apogee_ECI
        x0[3:6] = vel_apogee_ECI
        ode_log = odeint(payload_parachute_dynamics, x0, time, args=(rocket, ))

        date_array = rocket.launch_date + np.array([datetime.timedelta(seconds=sec) for sec in time])
        pos_ECEF = np.array([coord.DCM_ECI2ECEF(t).dot(pos) for pos, t in zip(ode_log[:, 0:3], time)])
        pos_LLH = np.array([pm.ecef2geodetic(pos[0], pos[1], pos[2]) for pos in pos_ECEF])

        # 着地後の分をカット
        index_payload_landing = np.argmax(pos_LLH[:, 2] <= 0.0)
        # log
        rocket.payload.result.time_decent_log = time[:index_payload_landing+1]
        rocket.payload.result.pos_decent_LLH_log = pos_LLH[:index_payload_landing+1, :]

        