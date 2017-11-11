import numpy as np
from scipy import interpolate
from scipy.integrate import odeint

import coordinate as coord
import environment as env

class Simulator:
    def __init__(self):
        self.time_log = []
        self.X_ENU_log = []
        self.Y_ENU_log = []
        self.Z_ENU_log = []
        self.Vx_ENU_log = []
        self.Vy_ENU_log = []
        self.Vz_ENU_log = []
        self.Vx_Body_log = []
        self.Vy_Body_log = []
        self.Vz_Body_log = []
        self.Accx_Body_log = []
        self.Accy_Body_log = []
        self.Accz_Body_log = []
        self.Vx_Air_log = []
        self.Vy_Air_log = []
        self.Vz_Air_log = []
        self.V_Air_log = []
        self.Mach_log = []
        self.dynamic_pressure_log = []
        self.Fx_log = []
        self.Fy_log = []
        self.Fz_log = []
        self.thrust_log = []
        self.Isp_log = []
        self.drag_log = []
        self.normaly_log = []
        self.normalz_log = []
        self.azimuth_log = []
        self.elevation_log = []
        self.roll_log = []
        self.alpha_log = []
        self.beta_log = []
        self.omegax_log = []
        self.omegay_log = []
        self.omegaz_log = []
        self.momentx_log = []
        self.momenty_log = []
        self.momentz_log = []
        self.m_log = []
        self.mp_log = []
        self.mox_log = []
        self.mf_log = []
        self.mdot_p_log = []
        self.mdot_ox_log = []
        self.mdot_f_log = []
        self.Lcg_log = []
        self.Lcg_ox_log = []
        self.Lcp_log = []
        self.g_log = []
        self.P_air_log = []
        self.rho_air_log = []

    def Angle_of_Attack(Vel_Air):
        Vel_Air_abs = np.linalg.norm(Vel_Air)
        u = Vel_Air_Bodyframe[0]
        v = Vel_Air_Bodyframe[1]
        w = Vel_Air_Bodyframe[2]

        if Vel_Air_abs < 0.0 or np.abs(u) < 0.0:
            alpha = 0.0
            beta = 0.0
        else:
            alpha = np.arctan2(w, u)
            beta = np.arcsin(v / Vel_Air_abs)

        return alpha, beta # rad


    def dynamics(x, t, rocket):
        Pos_ENU = np.array([x[0], x[1], x[2]])
        altitude = Pos_ENU[2]
        Vel_ENU = np.array([x[3], x[4], x[5]])
        Vel_Body = np.array([x[6], x[7], x[8]])
        omega_Body = np.array([x[9], x[10], x[11]])
        quat = np.array([x[12], x[13], x[14], x[15]])
        mf = x[16]
        mox = x[17]
        quat = coord.quat_normalize(quat)

        # Translation coordinate
        DCM_ENU2Body = coord.DCM_ENU2Body_quat(quat)
        DCM_Body2ENU = DCM_ENU2Body.transpose()

        # Attitude
        azimuth, elevation, roll = coord.quat2euler(DCM_ENU2Body)

        # alpha beta Vel_Air
        wind_NED = env.Wind_NED(rocket.WindSpeed, rocket.WindDirection, altitude, rocket.Wind_refAltitude, rocket.Wind_power_exp)
        Vel_air = DCM_ENU2Body.dot(Vel_ENU - wind_NED)
        Vel_air_abs = np.linalg.norm(Vel_air)
        alpha, beta = self.Angle_of_Attack(Vel_Air)

        # Air Condition
        g0 = 9.80665
        g = np.array([0.0, 0.0, -env.gravity(Altitude)])
        Ta0, Pa0, rho0, Cs0 = env.std_atmo(0.0)
        Ta, Pa, rho, Cs = env.std_atmo(altitude)
        Mach = Vel_air_abs / Cs
        dynamic_pressure = 0.5 * rho * Vel_air_abs ** 2

        # Mass
        # Limit Negative Propellant Mass
        if mf <= rocket.mf_after:
            mdot_f = 0.0
            mf = rocket.mf_after
        if mox <= 0.0:
            mdot_ox = 0.0
            mox = 0.0
        mp = mf + mox
        m = rocket.ms + mp

        if rocket.thrust(t) <= 0.0:
            thrust = np.zeros(3)
            Isp = 0.0
            mdot_p = 0.0
            mdot_f = 0.0
            mdot_ox = 0.0
        else:
            mdot_p = rocket.thrust(t) / (rocket.Isp * g0)
            mdot_f = rocket.mdot_f
            mdot_ox = mdot_p - mdot_f
            pressure_thrust = (Pa0 - Pa) * rocket.Ae            
            thrust = np.array([rocket.thrust(t) + pressure_thrust, 0.0, 0.0])
            Isp = rocket.Isp + pressure_thrust / (mdot_p * g0)
            # Limit Increase and Empty Oxidizers
            if mdot_ox < 0.0:
                mdot_ox = 0.0
        
        # Aero Force
        drag = dynamic_pressure * rocket.Cd(Mach) * rocket.A
        normal = dynamic_pressure * rocket.CNa(Mach) * rocket.A
        F_aero = np.array([-drag, normal * beta, -normal * alpha])

        # Newton Equation
        Acc_Body =  (thrust + F_aero) / m + DCM_ENU2Body(g)
        Acc_ENU = DCM_Body2ENU.dot(Acc_Body)

        # Center of Gravity
        Lcg_ox_fromend = rocket.L_motor + (mox / rocket.m0_ox) * (rocket.Lcg0_ox - rocket.L_motor)
        Lcg_ox = rocket.L - Lcg_ox_fromend
        Lcg_p = (mf * rocket.Lcg_f + mox * Lcg_ox) / mp
        Lcg = (mp * Lcg_p + rocket.ms * rocket.Lcg_s) / m
        Lcp = rocket.Lcp(Mach)

        # Fuel Inertia Moment
        Ij_f_pitch = mf * ((rocket.d_port_f ** 2 + rocket.d_out_f ** 2) / 4.0 + rocket.L_f / 3.0) / 4.0
        Ij_f_roll = mf * (rocket.d_port_f ** 2 + rocket.d_out_f ** 2) / 8.0

        # Offset Inertia Moment
        Ij_s_pitch = rocket.Ij0_s_pitch + rocket.ms * (Lcg - rocket.Lcg_s) ** 2
        Ij_f_pitch += mf * (Lcg - rocket.Lcg_f) ** 2
        Ij_pitch = Ij_s_pitch + Ij_f_pitch
        Ij_roll = Ij0_s_roll + Ij_f_roll
        Ij = np.array([Ij_roll, Ij_pitch, Ij_pitch])
        Ijdot_f = np.array([Ij_f_roll * mdot_f / mf, Ij_f_pitch * mdot_f / mf, Ij_f_pitch * mdot_f / mf])

        # Aero Moment
        moment_aero = np.array([0.0, F_aero[2] * (Lcp - Lcg), -F_aero[1] * (Lcp - Lcg)])

        # Aero Dumping Moment
        moment_aero_dumping = np.zeros(3)
        moment_aero_dumping[0] = dynamic_pressure * rocket.Clp * rocket.A * rocket.d ** 2 * 0.5 / Vel_air_abs * omega_Body[0]
        moment_aero_dumping[1] = dynamic_pressure * rocket.Cmq * rocket.A * rocket.L ** 2 * 0.5 / Vel_Air_abs * omega_Body[1]
        moment_aero_dumping[2] = dynamic_pressure * rocket.Cnr * rocket.A * rocket.L ** 2 * 0.5 / Vel_Air_abs * omega_Body[2]

        # Jet Dumping Moment
        moment_jet_dumping = np.zeros(3)
        moment_jet_dumping[0] = -(-Ijdot_f[0] + mdot_p * 0.5 * (0.25 * rocket.de ** 2)) * omega_Body[0]
        moment_jet_dumping[1] = -(-Ijdot_f[1] + mdot_p * ((Lcg - Lcgp) ** 2 - (rocket.L - Lcgp) ** 2)) * omega_Body[1]
        moment_jet_dumping[2] = -(-Ijdot_f[2] + mdot_p * ((Lcg - Lcgp) ** 2 - (rocket.L - Lcgp) ** 2)) * omega_Body[2]

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
        tersor = np.array(tersor_0, tersor_1, tersor_2, tersor_3)
        quatdot = 0.5 * tersor.dot(quat)

        dx = np.zeros(18)
        dx[0:3] = Vel_ENU  # Pos_ENU
        dx[3:6] = Acc_ENU  # Vel_ENU
        dx[6:9] = Acc_Body  # Vel_Body
        dx[9:12] =  omegadot  # omega_Body
        dx[12:16] = quatdot  # quat
        dx[16] = -mdot_f  # mf
        dx[17] = -mdot_ox  # mox

        return dx

    def simulation(rocket, vel_wind, angle_wind):
        

        x0 = np.zeros(18)
        x0[0] = rocket.Lcg0_s
        x0[1] = 0.0
        x0[2] = 0.0
        x0[3:6] = np.array([[0.0] * 3])
        x0[]

        dx[3:6] = Acc_ENU  # Vel_ENU
        dx[6:9] = Acc_Body  # Vel_Body
        dx[9:12] =  omegadot  # omega_Body
        dx[12:16] = quatdot  # quat
        dx[16] = -mdot_f  # mf
        dx[17] = -mdot_ox  # mox