import datetime
import numpy as np
import pymap3d as pm

import Simulator.coordinate as coord
import Simulator.environment as env


def _dynamics(pos_ECI, vel_ECI, omega_BODY, quat, t, rocket):

    date_current = rocket.launch_date + datetime.timedelta(seconds=t)
    
    # Translation coordinate
    DCM_NED2BODY = coord.DCM_NED2BODY_quat(quat)
    DCM_BODY2NED = DCM_NED2BODY.transpose()
    DCM_ECEF2NED = coord.DCM_ECEF2NED(rocket.pos0_LLH)
    DCM_NED2ECEF = DCM_ECEF2NED.transpose()
    DCM_ECI2ECEF = coord.DCM_ECI2ECEF(t)
    DCM_ECEF2ECI = DCM_ECI2ECEF.transpose()
    DCM_BODY2ECI = DCM_ECEF2ECI.dot(DCM_NED2ECEF.dot(DCM_BODY2NED))

    pos_ECEF = DCM_ECI2ECEF.dot(pos_ECI)
    pos_LLH = pm.ecef2geodetic(pos_ECEF[0], pos_ECEF[1], pos_ECEF[2])
    # pos_LLH = pm.eci2geodetic(pos_ECI, date_current)

    vel_ECEF = coord.vel_ECI2ECEF(vel_ECI, DCM_ECI2ECEF, pos_ECI)
    vel_NED = DCM_ECEF2NED.dot(vel_ECEF)

    altitude = pos_LLH[2]
    g0 = 9.80665

    # alpha beta vel_Air
    wind_NED = env.Wind_NED(rocket.wind_speed(altitude), rocket.wind_direction(altitude))    
    vel_AIR_BODYframe = DCM_NED2BODY.dot(vel_NED - wind_NED)
    vel_AIR_BODYframe_abs = np.linalg.norm(vel_AIR_BODYframe)
    if vel_AIR_BODYframe_abs <= 0.0 or np.abs(vel_AIR_BODYframe[0]) <= 0.0:
        alpha = 0.0
        beta = 0.0
    else:
        alpha = np.arctan2(vel_AIR_BODYframe[2], vel_AIR_BODYframe[0])
        beta = np.arcsin(-vel_AIR_BODYframe[1] / vel_AIR_BODYframe_abs)

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
    F_aero_WIND = coord.DCM_WIND2BODY(alpha, beta).transpose().dot(F_aero_BODY)

    # Newton Equation
    g_NED = np.array([0.0, 0.0, env.gravity(altitude)])
    force_BODY = F_aero_BODY + np.array([thrust, 0, 0])
    acc_BODY =  force_BODY / m + DCM_NED2BODY.dot(g_NED)
    acc_ECI = DCM_BODY2ECI.dot(acc_BODY)

    # Center of Gravity
    Lcg_p = rocket.Lcg_p(t)
    Lcg = rocket.Lcg(t)
    Lcp = rocket.Lcp(Mach)
    moment_arm = np.array([Lcg - Lcp, 0, 0])

    # Inertia Moment
    Ij_pitch = rocket.Ij_pitch(t)
    Ij_roll = rocket.Ij_roll(t)
    Ij = np.array([Ij_roll, Ij_pitch, Ij_pitch])

    Ijdot_f_pitch = rocket.Ijdot_f_pitch(t)
    Ijdot_f_roll = rocket.Ijdot_f_roll(t)
    Ijdot_f = np.array([Ijdot_f_roll, Ijdot_f_pitch, Ijdot_f_pitch])

    # Aero Moment
    moment_aero = np.cross(F_aero_BODY, moment_arm)

    # Aero Dumping Moment
    moment_aero_dumping = np.zeros(3)
    moment_aero_dumping[0] = dynamic_pressure * rocket.Clp * rocket.A * rocket.d ** 2 * 0.5 / vel_AIR_BODYframe_abs * omega_BODY[0]
    moment_aero_dumping[1] = dynamic_pressure * rocket.Cmq * rocket.A * rocket.L ** 2 * 0.5 / vel_AIR_BODYframe_abs * omega_BODY[1]
    moment_aero_dumping[2] = dynamic_pressure * rocket.Cnr * rocket.A * rocket.L ** 2 * 0.5 / vel_AIR_BODYframe_abs * omega_BODY[2]

    # Jet Dumping Moment
    moment_jet_dumping = np.zeros(3)
    moment_jet_dumping[0] = (-Ijdot_f[0] + mdot_p * 0.5 * (0.25 * rocket.de ** 2)) * omega_BODY[0]
    moment_jet_dumping[1] = (-Ijdot_f[1] + mdot_p * ((Lcg - Lcg_p) ** 2 - (rocket.L - Lcg_p) ** 2)) * omega_BODY[1]
    moment_jet_dumping[2] = (-Ijdot_f[2] + mdot_p * ((Lcg - Lcg_p) ** 2 - (rocket.L - Lcg_p) ** 2)) * omega_BODY[2]
    # moment_jet_dumping[0] = -(mdot_p * 0.5 * (0.25 * rocket.de ** 2)) * omega_BODY[0]
    # moment_jet_dumping[0] = 0.0
    # moment_jet_dumping[1] = -(mdot_p * ((Lcg - Lcg_p) ** 2 - (rocket.L - Lcg_p) ** 2)) * omega_BODY[1]
    # moment_jet_dumping[2] = -(mdot_p * ((Lcg - Lcg_p) ** 2 - (rocket.L - Lcg_p) ** 2)) * omega_BODY[2]

    # Euler Equation
    moment = moment_aero + moment_aero_dumping + moment_jet_dumping
    p = omega_BODY[0]
    q = omega_BODY[1]
    r = omega_BODY[2]
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

    output_data = [date_current, pos_ECEF, pos_LLH, vel_ECEF, vel_NED,
                    vel_AIR_BODYframe, alpha, beta, Ta, Pa, rho, Cs, Mach, dynamic_pressure,
                    mdot_p, mdot_f, mdot_ox, thrust, Isp, mf, mox, mp, m,
                    drag, normal, F_aero_WIND, F_aero_BODY, g_NED, force_BODY, acc_BODY, acc_ECI,
                    Lcg_p, Lcg, Lcp, Ij, Ijdot_f, moment_aero, moment_aero_dumping, moment_jet_dumping, moment]

    return vel_ECI, acc_ECI, omegadot, quatdot, output_data


def dynamics_odeint(x, t, rocket):
    pos_ECI = x[0:3]
    vel_ECI = x[3:6]
    omega_BODY = x[6:9]
    x[9:13] = coord.quat_normalize(x[9:13])
    quat = x[9:13]

    vel_ECI, acc_ECI, omegadot, quatdot, _ = _dynamics(pos_ECI, vel_ECI, omega_BODY, quat, t, rocket)

    dx = np.zeros(13)
    dx[0:3] = vel_ECI  # pos_ECI
    dx[3:6] = acc_ECI  # vel_ECI
    dx[6:9] = omegadot  # omega_BODY
    dx[9:13] = quatdot  # quat
    return dx


def dynamics_result(rocket):
    date_log = []
    pos_ECEF_log = []
    pos_LLH_log = []
    vel_ECEF_log = []
    vel_NED_log = []
    vel_AIR_BODYframe_log = []
    alpha_log = []
    beta_log = []
    Ta_log = []
    Pa_log = []
    rho_log = []
    Cs_log = []
    Mach_log = []
    dynamic_pressure_log = []
    mdot_p_log = []
    mdot_f_log = []
    mdot_ox_log = []
    thrust_log = []
    Isp_log = []
    mf_log = []
    mox_log = []
    mp_log = []
    m_log = []
    drag_log = []
    normal_log = []
    F_aero_WIND_log = []
    F_aero_BODY_log = []
    g_NED_log = []
    force_BODY_log = []
    acc_BODY_log = []
    acc_ECI_log = []
    Lcg_p_log = []
    Lcg_log = []
    Lcp_log = []
    Ij_log = []
    Ijdot_f_log = []
    moment_aero_log = []
    moment_aero_dumping_log = []
    moment_jet_dumping_log = []
    moment_log = []
    omegadot_log = []

    for i in range(len(rocket.result.time_log)):
        vel_ECI, acc_ECI, omegadot, quatdot, output_data = _dynamics(rocket.result.pos_ECI_log[i], rocket.result.vel_ECI_log[i], 
                                                                    rocket.result.omega_log[i], rocket.result.quat_log[i], rocket.result.time_log[i], 
                                                                    rocket)
        date_log.append(output_data[0])
        pos_ECEF_log.append(output_data[1])
        pos_LLH_log.append(output_data[2])
        vel_ECEF_log.append(output_data[3])
        vel_NED_log.append(output_data[4])
        vel_AIR_BODYframe_log.append(output_data[5])
        alpha_log.append(output_data[6])
        beta_log.append(output_data[7])
        Ta_log.append(output_data[8])
        Pa_log.append(output_data[9])
        rho_log.append(output_data[10])
        Cs_log.append(output_data[11])
        Mach_log.append(output_data[12])
        dynamic_pressure_log.append(output_data[13])
        mdot_p_log.append(output_data[14])
        mdot_f_log.append(output_data[15])
        mdot_ox_log.append(output_data[16])
        thrust_log.append(output_data[17])
        Isp_log.append(output_data[18])
        mf_log.append(output_data[19])
        mox_log.append(output_data[20])
        mp_log.append(output_data[21])
        m_log.append(output_data[22])
        drag_log.append(output_data[23])
        normal_log.append(output_data[24])
        F_aero_WIND_log.append(output_data[25])
        F_aero_BODY_log.append(output_data[26])
        g_NED_log.append(output_data[27])
        force_BODY_log.append(output_data[28])
        acc_BODY_log.append(output_data[29])
        acc_ECI_log.append(output_data[30])
        Lcg_p_log.append(output_data[31])
        Lcg_log.append(output_data[32])
        Lcp_log.append(output_data[33])
        Ij_log.append(output_data[34])
        Ijdot_f_log.append(output_data[35])
        moment_aero_log.append(output_data[36])
        moment_aero_dumping_log.append(output_data[37])
        moment_jet_dumping_log.append(output_data[38])
        moment_log.append(output_data[39])
        omegadot_log.append(omegadot)
        
    rocket.result.date_log = np.array(date_log)
    rocket.result.pos_ECEF_log = np.array(pos_ECEF_log)
    rocket.result.pos_LLH_log = np.array(pos_LLH_log)
    rocket.result.vel_ECEF_log = np.array(vel_ECEF_log)
    rocket.result.vel_NED_log = np.array(vel_NED_log)
    rocket.result.vel_AIR_BODYframe_log = np.array(vel_AIR_BODYframe_log)
    rocket.result.alpha_log = np.array(alpha_log)
    rocket.result.beta_log = np.array(beta_log)
    rocket.result.Ta_log = np.array(Ta_log)
    rocket.result.Pa_log = np.array(Pa_log)
    rocket.result.rho_log = np.array(rho_log)
    rocket.result.Cs_log = np.array(Cs_log)
    rocket.result.Mach_log = np.array(Mach_log)
    rocket.result.dynamic_pressure_log = np.array(dynamic_pressure_log)
    rocket.result.mdot_p_log = np.array(mdot_p_log)
    rocket.result.mdot_f_log = np.array(mdot_f_log)
    rocket.result.mdot_ox_log = np.array(mdot_ox_log)
    rocket.result.thrust_log = np.array(thrust_log)
    rocket.result.Isp_log = np.array(Isp_log)
    rocket.result.mf_log = np.array(mf_log)
    rocket.result.mox_log = np.array(mox_log)
    rocket.result.mp_log = np.array(mp_log)
    rocket.result.m_log = np.array(m_log)
    rocket.result.drag_log = np.array(drag_log)
    rocket.result.normal_log = np.array(normal_log)
    rocket.result.F_aero_WIND_log = np.array(F_aero_WIND_log)
    rocket.result.F_aero_BODY_log = np.array(F_aero_BODY_log)
    rocket.result.g_NED_log = np.array(g_NED_log)
    rocket.result.force_BODY_log = np.array(force_BODY_log)
    rocket.result.acc_BODY_log = np.array(acc_BODY_log)
    rocket.result.acc_ECI_log = np.array(acc_ECI_log)
    rocket.result.Lcg_p_log = np.array(Lcg_p_log)
    rocket.result.Lcg_log = np.array(Lcg_log)
    rocket.result.Lcp_log = np.array(Lcp_log)
    rocket.result.Ij_log = np.array(Ij_log)
    rocket.result.Ijdot_f_log = np.array(Ijdot_f_log)
    rocket.result.moment_aero_log = np.array(moment_aero_log)
    rocket.result.moment_aero_dumping_log = np.array(moment_aero_dumping_log)
    rocket.result.moment_jet_dumping_log = np.array(moment_jet_dumping_log)
    rocket.result.moment_log = np.array(moment_log)
            