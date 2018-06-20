import numpy as np
from scipy.integrate import odeint

import Simulator.coordinate as coord
import Simulator.environment as env


def dynamics(x, t, rocket, launch_site):
    Pos_ENU = x[0:3]
    Vel_ENU = x[3:6]
    omega_Body = x[6:9]
    quat = x[9:13]

    altitude = Pos_ENU[2]
    quat = coord.quat_normalize(quat)

    # Translation coordinate
    DCM_ENU2Body = coord.DCM_ENU2Body_quat(quat)
    DCM_Body2ENU = DCM_ENU2Body.transpose()

    # Attitude
    # azimuth, elevation, roll = coord.quat2euler(DCM_ENU2Body)

    # alpha beta Vel_Air
    wind_ENU = env.Wind_ENU(launch_site.wind_speed(altitude), launch_site.wind_direction(altitude))      
    Vel_air = DCM_ENU2Body.dot(Vel_ENU - wind_ENU)
    Vel_air_abs = np.linalg.norm(Vel_air)
    u = Vel_air[0]
    v = Vel_air[1]
    w = Vel_air[2]

    if Vel_air_abs < 0.0 or np.abs(u) < 0.0:
        alpha = 0.0
        beta = 0.0
    else:
        alpha = np.arctan2(w, u)
        beta = np.arcsin(-v / Vel_air_abs)

    # Air Condition
    g0 = 9.80665
    g = np.array([0.0, 0.0, -env.gravity(altitude)])
    Ta0, Pa0, rho0, Cs0 = env.std_atmo(0.0)
    Ta, Pa, rho, Cs = env.std_atmo(altitude)
    Mach = Vel_air_abs / Cs
    dynamic_pressure = 0.5 * rho * Vel_air_abs ** 2

    # Mass
    if rocket.thrust(t) <= 0.0:
        thrust = np.zeros(3)
        Isp = 0.0
        mdot_p = 0.0
        mdot_f = 0.0
        mdot_ox = 0.0
    else:
        mdot_p = rocket.mdot_p(t)
        mdot_f = rocket.mdot_f(t)
        mdot_ox = rocket.mdot_ox(t)
        pressure_thrust = (Pa0 - Pa) * rocket.Ae
        thrust = np.array([rocket.thrust(t) + pressure_thrust, 0.0, 0.0])
        Isp = rocket.Isp + pressure_thrust / (mdot_p * g0)

    mf = rocket.mf(t)
    mox = rocket.mox(t)
    mp = mf + mox
    m = rocket.ms + mp

    # Aero Force
    drag = dynamic_pressure * rocket.Cd(Mach) * rocket.A
    normal = dynamic_pressure * rocket.CNa(Mach) * rocket.A
    F_aero = np.array([-drag, normal * beta, -normal * alpha])

    # Newton Equation
    Force = (thrust + F_aero)
    Acc_ENU = DCM_Body2ENU.dot(Force) / m + g

    # Center of Gravity
    Lcg_p = rocket.Lcg_p(t)
    Lcg = rocket.Lcg(t)
    Lcp = rocket.Lcp(Mach)

    # Inertia Moment
    Ij_pitch = rocket.Ij_pitch(t)
    Ij_roll = rocket.Ij_roll(t)
    Ij = np.array([Ij_roll, Ij_pitch, Ij_pitch])

    Ijdot_f_pitch = rocket.Ijdot_f_pitch(t)
    Ijdot_f_roll = rocket.Ijdot_f_roll(t)
    Ijdot_f = np.array([Ijdot_f_roll, Ijdot_f_pitch, Ijdot_f_pitch])

    # Aero Moment
    moment_aero = np.array([0.0, F_aero[2] * (Lcp - Lcg), -F_aero[1] * (Lcp - Lcg)])

    # Aero Dumping Moment
    moment_aero_dumping = np.zeros(3)
    moment_aero_dumping[0] = dynamic_pressure * rocket.Clp * rocket.A * rocket.d ** 2 * 0.5 / Vel_air_abs * omega_Body[0]
    moment_aero_dumping[1] = dynamic_pressure * rocket.Cmq * rocket.A * rocket.L ** 2 * 0.5 / Vel_air_abs * omega_Body[1]
    moment_aero_dumping[2] = dynamic_pressure * rocket.Cnr * rocket.A * rocket.L ** 2 * 0.5 / Vel_air_abs * omega_Body[2]

    # Jet Dumping Moment
    moment_jet_dumping = np.zeros(3)
    moment_jet_dumping[0] = (-Ijdot_f[0] + mdot_p * 0.5 * (0.25 * rocket.de ** 2)) * omega_Body[0]
    moment_jet_dumping[1] = (-Ijdot_f[1] + mdot_p * ((Lcg - Lcg_p) ** 2 - (rocket.L - Lcg_p) ** 2)) * omega_Body[1]
    moment_jet_dumping[2] = (-Ijdot_f[2] + mdot_p * ((Lcg - Lcg_p) ** 2 - (rocket.L - Lcg_p) ** 2)) * omega_Body[2]
    # moment_jet_dumping[0] = -(mdot_p * 0.5 * (0.25 * rocket.de ** 2)) * omega_Body[0]
    # moment_jet_dumping[0] = 0.0
    # moment_jet_dumping[1] = -(mdot_p * ((Lcg - Lcg_p) ** 2 - (rocket.L - Lcg_p) ** 2)) * omega_Body[1]
    # moment_jet_dumping[2] = -(mdot_p * ((Lcg - Lcg_p) ** 2 - (rocket.L - Lcg_p) ** 2)) * omega_Body[2]

    # Euler Equation
    moment = moment_aero + moment_aero_dumping + moment_jet_dumping
    p = omega_Body[0]
    q = omega_Body[1]
    r = omega_Body[2]
    omegadot = np.zeros(3)
    omegadot[0] = ((Ij[1] - Ij[2]) * q * r + moment[0]) / Ij[0]
    omegadot[1] = ((Ij[2] - Ij[0]) * p * r + moment[1]) / Ij[1]
    omegadot[2] = ((Ij[0] - Ij[1]) * p * q + moment[2]) / Ij[2]

    # Kinematic Equation
    tersor_0 = [0.0, r, -q, p]
    tersor_1 = [-r, 0.0, p, q]
    tersor_2 = [q, -p, 0.0, r]
    tersor_3 = [-p, -q, -r, 0.0]
    tersor = np.array([tersor_0, tersor_1, tersor_2, tersor_3])
    quatdot = 0.5 * tersor.dot(quat)

    dx = np.zeros(13)
    dx[0:3] = Vel_ENU  # Pos_ENU
    dx[3:6] = Acc_ENU  # Vel_ENU
    dx[6:9] = omegadot  # omega_Body
    dx[9:13] = quatdot  # quat

    return dx

def onlauncher_dynamics(x, t, rocket, launch_site, quat0):
    Pos_ENU = x[0:3]
    altitude = Pos_ENU[2]
    Vel_ENU = x[6:9]
    Vel_Body = x[9:12]
    mf = rocket.mf(t)
    mox = rocket.mox(t)
    mp = mf + mox
    m = rocket.ms + mp

    # Translation coordinate
    DCM_ENU2Body = coord.DCM_ENU2Body_quat(quat0)
    DCM_Body2ENU = DCM_ENU2Body.transpose()

    # alpha beta Vel_Air
    wind_ENU = env.Wind_ENU(launch_site.wind_speed(altitude), launch_site.wind_direction(altitude))
    Vel_air = DCM_ENU2Body.dot(Vel_ENU - wind_ENU)
    Vel_air_abs = np.linalg.norm(Vel_air)
    u = Vel_air[0]
    v = Vel_air[1]
    w = Vel_air[2]

    # Air Condition
    g0 = 9.80665
    g = np.array([0.0, 0.0, -env.gravity(altitude)])
    Ta0, Pa0, rho0, Cs0 = env.std_atmo(0.0)
    Ta, Pa, rho, Cs = env.std_atmo(altitude)
    Mach = Vel_air_abs / Cs
    dynamic_pressure = 0.5 * rho * Vel_air_abs ** 2

    # Mass
    mdot_p = rocket.mdot_p(t)
    mdot_f = rocket.mdot_f(t)
    mdot_ox = rocket.mdot_ox(t)
    pressure_thrust = (Pa0 - Pa) * rocket.Ae
    thrust = np.array([rocket.thrust(t) + pressure_thrust, 0.0, 0.0])
    Isp = rocket.Isp + pressure_thrust / (mdot_p * g0)

    # Aero Force
    drag = dynamic_pressure * rocket.Cd(Mach) * rocket.A
    normal = dynamic_pressure * rocket.CNa(Mach) * rocket.A
    F_aero = np.array([-drag, 0.0, 0.0])

    # Newton Equation
    Force = (thrust + F_aero)
    Acc_Body =  Force / m + DCM_ENU2Body.dot(g)
    Acc_ENU = DCM_Body2ENU.dot(Force) / m + g

    dx = np.zeros(12)
    dx[0:3] = Vel_ENU  # Pos_ENU
    dx[3:6] = Vel_Body  # distance_Body
    dx[6:9] = Acc_ENU  # Vel_ENU
    dx[9:12] = Acc_Body  # Vel_Body

    return dx

def onlauncher_tipoff_dynamics(x, t, rocket, launch_site, launcher_rail):
    Pos_ENU = x[0:3]
    altitude = Pos_ENU[2]
    distance_Body = x[3]
    Vel_ENU = x[6:9]
    Vel_Body = x[9:12]
    omega_Body = x[12:15]
    quat = x[15:19]

    quat = coord.quat_normalize(quat)

    mf = rocket.mf(t)
    mox = rocket.mox(t)
    mp = mf + mox
    m = rocket.ms + mp

    # Translation coordinate
    DCM_ENU2Body = coord.DCM_ENU2Body_quat(quat)
    DCM_Body2ENU = DCM_ENU2Body.transpose()

    # alpha beta Vel_Air
    wind_ENU = env.Wind_ENU(launch_site.wind_speed(altitude), launch_site.wind_direction(altitude))
    Vel_air = DCM_ENU2Body.dot(Vel_ENU - wind_ENU)
    Vel_air_abs = np.linalg.norm(Vel_air)
    u = Vel_air[0]
    v = Vel_air[1]
    w = Vel_air[2]

    if Vel_air_abs < 0.0 or np.abs(u) < 0.0:
        alpha = 0.0
        beta = 0.0
    else:
        alpha = np.arctan2(w, u)
        beta = np.arcsin(-v / Vel_air_abs)

    # Air Condition
    g0 = 9.80665
    g = np.array([0.0, 0.0, -env.gravity(altitude)])
    Ta0, Pa0, rho0, Cs0 = env.std_atmo(0.0)
    Ta, Pa, rho, Cs = env.std_atmo(altitude)
    Mach = Vel_air_abs / Cs
    dynamic_pressure = 0.5 * rho * Vel_air_abs ** 2

    # Mass
    mdot_p = rocket.mdot_p(t)
    mdot_f = rocket.mdot_f(t)
    mdot_ox = rocket.mdot_ox(t)
    pressure_thrust = (Pa0 - Pa) * rocket.Ae
    thrust = np.array([rocket.thrust(t) + pressure_thrust, 0.0, 0.0])
    Isp = rocket.Isp + pressure_thrust / (mdot_p * g0)

    # Aero Force
    drag = dynamic_pressure * rocket.Cd(Mach) * rocket.A
    normal = dynamic_pressure * rocket.CNa(Mach) * rocket.A
    F_aero = np.array([-drag, normal * beta, -normal * alpha])


    # Newton Equation
    Force = (thrust + F_aero)
    Acc_Body =  Force / m + DCM_ENU2Body.dot(g)
    Acc_ENU = DCM_Body2ENU.dot(Force) / m + g

    # Center of Gravity
    Lcg_p = rocket.Lcg_p(t)    
    Lcg = rocket.Lcg(t)
    Lcp = rocket.Lcp(Mach)

    # omega
    p = omega_Body[0]
    q = omega_Body[1]
    r = omega_Body[2]
    omegadot = np.zeros(3)  # 0埋めしてるので上部ラグがレール上では0のまま

    distance_upper_lug = (rocket.L - rocket.upper_lug) + distance_Body  # 下端ラグの機体後端からのオフセット
    distance_lower_lug = (rocket.L - rocket.lower_lug) + distance_Body
    if distance_upper_lug > launcher_rail:
        if distance_lower_lug > launcher_rail: 
            pivot_point = rocket.lower_lug  # ランチクリア後
        else:
            pivot_point = Lcg  # 上部ラグがランチクリア
        
        # Inertia Moment
        Ij_pitch = rocket.Ij_pitch(t) + m * np.abs(Lcg - pivot_point) ** 2
        Ij_roll = rocket.Ij_roll(t)
        Ij = np.array([Ij_roll, Ij_pitch, Ij_pitch])

        Ijdot_f_pitch = rocket.Ijdot_f_pitch(t) + m * np.abs(Lcg - pivot_point) ** 2
        Ijdot_f_roll = rocket.Ijdot_f_roll(t)
        Ijdot_f = np.array([Ijdot_f_roll, Ijdot_f_pitch, Ijdot_f_pitch])

        # Aero Moment
        moment_aero = np.array([0.0, F_aero[2] * (Lcp - pivot_point), -F_aero[1] * (Lcp - pivot_point)])

        # Aero Dumping Moment
        moment_aero_dumping = np.zeros(3)
        moment_aero_dumping[0] = dynamic_pressure * rocket.Clp * rocket.A * rocket.d ** 2 * 0.5 / Vel_air_abs * omega_Body[0]
        moment_aero_dumping[1] = dynamic_pressure * rocket.Cmq * rocket.A * rocket.L ** 2 * 0.5 / Vel_air_abs * omega_Body[1]
        moment_aero_dumping[2] = dynamic_pressure * rocket.Cnr * rocket.A * rocket.L ** 2 * 0.5 / Vel_air_abs * omega_Body[2]

        # Jet Dumping Moment
        moment_jet_dumping = np.zeros(3)
        moment_jet_dumping[0] = (-Ijdot_f[0] + mdot_p * 0.5 * (0.25 * rocket.de ** 2)) * omega_Body[0]
        moment_jet_dumping[1] = (-Ijdot_f[1] + mdot_p * ((pivot_point - Lcg_p) ** 2 - (rocket.L - Lcg_p) ** 2)) * omega_Body[1]
        moment_jet_dumping[2] = (-Ijdot_f[2] + mdot_p * ((pivot_point - Lcg_p) ** 2 - (rocket.L - Lcg_p) ** 2)) * omega_Body[2]

        # Euler Equation
        moment = moment_aero + moment_aero_dumping + moment_jet_dumping

        omegadot[0] = ((Ij[1] - Ij[2]) * q * r + moment[0]) / Ij[0]
        omegadot[1] = ((Ij[2] - Ij[0]) * p * r + moment[1]) / Ij[1]
        omegadot[2] = ((Ij[0] - Ij[1]) * p * q + moment[2]) / Ij[2]

    # Kinematic Equation
    tersor_0 = [0.0, r, -q, p]
    tersor_1 = [-r, 0.0, p, q]
    tersor_2 = [q, -p, 0.0, r]
    tersor_3 = [-p, -q, -r, 0.0]
    tersor = np.array([tersor_0, tersor_1, tersor_2, tersor_3])
    quatdot = 0.5 * tersor.dot(quat)


    dx = np.zeros(19)
    dx[0:3] = Vel_ENU  # Pos_ENU
    dx[3:6] = Vel_Body  # distance_Body
    dx[6:9] = Acc_ENU  # Vel_ENU
    dx[9:12] = Acc_Body  # Vel_Body
    dx[12:15] = omegadot  # omega_Body
    dx[15:19] = quatdot  # quat

    return dx


def parachute_dynamics(x, t, rocket, launch_site, m_apogee):
    m = m_apogee
    Pos_ENU = x[0:3]
    altitude = Pos_ENU[2]
    Vel_descent = x[3]

    wind_ENU = env.Wind_ENU(launch_site.wind_speed(altitude), launch_site.wind_direction(altitude))    
    Vel_ENU = np.array([wind_ENU[0], wind_ENU[1], Vel_descent])

    g = env.gravity(altitude)
    Ta, Pa, rho, Cs = env.std_atmo(altitude)
    if rocket.timer_mode:
        if t > rocket.t_2nd_max or (altitude <= rocket.alt_sepa2 and t > rocket.t_2nd_min):
            CdS = rocket.CdS1 + rocket.CdS2
        else:
            CdS = rocket.CdS1
    else:
        CdS = rocket.CdS1 + rocket.CdS2 if altitude <= rocket.alt_sepa2 else rocket.CdS1
    drag = 0.5 * rho * CdS * Vel_ENU[2] ** 2
    Acc = drag / m - g
    dx = np.zeros(4)
    dx[0:3] = Vel_ENU  # Pos_ENU
    dx[3] = Acc  # Vel_descent

    return dx
