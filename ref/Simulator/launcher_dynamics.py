import numpy as np

import Simulator.coordinate as coord
import Simulator.environment as env


def onlauncher_dynamics(x, t, rocket, DCM_NED2LAUNCHER):
    pos_NED = x[0:3]
    vel_NED = x[3:6]
    omega_BODY = x[6:9]
    x[9:13] = coord.quat_normalize(x[9:13])
    quat = x[9:13]

    altitude = -pos_NED[2]
    g0 = 9.80665

    # Translation coordinate
    DCM_NED2BODY = coord.DCM_NED2BODY_quat(quat)
    DCM_BODY2NED = DCM_NED2BODY.transpose()
    DCM_LAUNCHER2NED = DCM_NED2LAUNCHER.transpose()
    DCM_BODY2LAUNCHER = DCM_NED2LAUNCHER.dot(DCM_BODY2NED)

    # alpha beta vel_Air
    wind_NED = env.Wind_NED(rocket.wind_speed(altitude), rocket.wind_direction(altitude))
    vel_AIR_BODYframe = DCM_NED2BODY.dot(vel_NED - wind_NED)
    vel_AIR_BODYframe_abs = np.linalg.norm(vel_AIR_BODYframe)
    if vel_AIR_BODYframe_abs <= 0.0 or np.abs(vel_AIR_BODYframe[0]) <= 0.0:
        alpha = 0.0
        beta = 0.0
    else:
        alpha = np.arctan2(vel_AIR_BODYframe[2], vel_AIR_BODYframe[0])
        beta = np.arcsin(vel_AIR_BODYframe[1] / vel_AIR_BODYframe_abs)

    # Air Condition
    Ta0, Pa0, rho0, Cs0 = env.std_atmo(0.0)
    Ta, Pa, rho, Cs = env.std_atmo(altitude)
    Mach = vel_AIR_BODYframe_abs / Cs
    dynamic_pressure = 0.5 * rho * vel_AIR_BODYframe_abs ** 2

    # Mass
    if rocket.thrust(t) <= 0.0:
        thrust = 0.0
        Isp = 0.0
        mdot_p = 0.0
        mdot_f = 0.0
        mdot_ox = 0.0
    else:
        mdot_p = rocket.mdot_p(t)
        mdot_f = rocket.mdot_f(t)
        mdot_ox = rocket.mdot_ox(t)
        pressure_thrust = (Pa0 - Pa) * rocket.Ae
        thrust = rocket.thrust(t) + pressure_thrust
        Isp = rocket.Isp + pressure_thrust / (mdot_p * g0)

    mf = rocket.mf(t)
    mox = rocket.mox(t)
    mp = mf + mox
    m = rocket.ms + mp

    # Aero Force
    drag = dynamic_pressure * rocket.Cd(Mach) * rocket.A
    normal = dynamic_pressure * rocket.CNa(Mach) * rocket.A
    F_aero_WIND = np.array([-drag, 0, 0])
    F_aero_BODY = coord.DCM_WIND2BODY(alpha, beta).dot(F_aero_WIND) + np.array([0, normal * beta, -normal * alpha])

    # Newton Equation
    g_NED = np.array([0.0, 0.0, env.gravity(altitude)])
    force_BODY = F_aero_BODY + np.array([thrust, 0, 0])
    acc_BODY =  force_BODY / m + DCM_NED2BODY.dot(g_NED)
    acc_LAUNCHER = DCM_BODY2LAUNCHER.dot(acc_BODY) * np.array([1.0, 0.0, 0.0])  # ランチャレール上ではランチャ軸以外0
    acc_NED = DCM_LAUNCHER2NED.dot(acc_LAUNCHER)

    # Center of Gravity
    Lcg_p = rocket.Lcg_p(t)    
    Lcg = rocket.Lcg(t)
    Lcp = rocket.Lcp(Mach)

    # omega
    p = omega_BODY[0]
    q = omega_BODY[1]
    r = omega_BODY[2]
    omegadot = np.zeros(3)  # 0埋めしてるので上部ラグがレール上では0のまま

    pos_upper_lug_LAUNCHER = DCM_NED2LAUNCHER.dot(pos_NED + DCM_BODY2NED.dot(np.array([rocket.upper_lug - Lcg, 0, 0])))
    pos_lower_lug_LAUNCHER = DCM_NED2LAUNCHER.dot(pos_NED + DCM_BODY2NED.dot(np.array([rocket.lower_lug - Lcg, 0, 0])))
    if pos_upper_lug_LAUNCHER[0] >= rocket.launcher_rail:
        if pos_lower_lug_LAUNCHER[0] >= rocket.launcher_rail: 
            pivot_point = rocket.lower_lug  # ランチクリア後
        else:
            pivot_point = Lcg  # 上部ラグのみクリア

        # Inertia Moment
        Ij_pitch = rocket.Ij_pitch(t) + m * np.abs(Lcg - pivot_point) ** 2
        Ij_roll = rocket.Ij_roll(t)
        Ij = np.array([Ij_roll, Ij_pitch, Ij_pitch])

        Ijdot_f_pitch = rocket.Ijdot_f_pitch(t) + m * np.abs(Lcg - pivot_point) ** 2
        Ijdot_f_roll = rocket.Ijdot_f_roll(t)
        Ijdot_f = np.array([Ijdot_f_roll, Ijdot_f_pitch, Ijdot_f_pitch])

        moment_arm = np.array([pivot_point - Lcp, 0, 0])

        # Aero Moment
        moment_aero = np.cross(F_aero_BODY, moment_arm)
        
        # Aero Dumping Moment
        moment_aero_dumping = np.zeros(3)
        # moment_aero_dumping[0] = dynamic_pressure * rocket.Clp * rocket.A * rocket.d ** 2 * 0.5 / vel_AIR_abs * omega_BODY[0]
        # moment_aero_dumping[1] = dynamic_pressure * rocket.Cmq * rocket.A * rocket.L ** 2 * 0.5 / vel_AIR_abs * omega_BODY[1]
        moment_aero_dumping[2] = dynamic_pressure * rocket.Cnr * rocket.A * rocket.L ** 2 * 0.5 / vel_AIR_BODYframe_abs * omega_BODY[2]

        # Jet Dumping Moment
        moment_jet_dumping = np.zeros(3)
        # moment_jet_dumping[0] = (-Ijdot_f[0] + mdot_p * 0.5 * (0.25 * rocket.de ** 2)) * omega_BODY[0]
        # moment_jet_dumping[1] = (-Ijdot_f[1] + mdot_p * ((pivot_point - Lcg_p) ** 2 - (rocket.L - Lcg_p) ** 2)) * omega_BODY[1]
        moment_jet_dumping[2] = (-Ijdot_f[2] + mdot_p * ((pivot_point - Lcg_p) ** 2 - (rocket.L - Lcg_p) ** 2)) * omega_BODY[2]

        # Euler Equation
        moment = moment_aero + moment_aero_dumping + moment_jet_dumping

        # omegadot[0] = ((Ij[1] - Ij[2]) * q * r + moment[0]) / Ij[0]  # ランチクリア後はpitch回転も許容するがランチクリア後はsolverでカットするため計算せず
        # omegadot[1] = ((Ij[2] - Ij[0]) * p * r + moment[1]) / Ij[1]
        omegadot[2] = ((Ij[0] - Ij[1]) * p * q + moment[2]) / Ij[2]  # Yawのみ許容

    # Kinematic Equation
    tersor_0 = [0.0, r, -q, p]
    tersor_1 = [-r, 0.0, p, q]
    tersor_2 = [q, -p, 0.0, r]
    tersor_3 = [-p, -q, -r, 0.0]
    tersor = np.array([tersor_0, tersor_1, tersor_2, tersor_3])
    quatdot = 0.5 * tersor.dot(quat)


    dx = np.zeros(13)
    dx[0:3] = vel_NED  # pos_NED
    dx[3:6] = acc_NED  # vel_NED
    dx[6:9] = omegadot  # omega_BODY
    dx[9:13] = quatdot  # quat

    return dx
