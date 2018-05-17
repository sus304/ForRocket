import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate
from scipy.integrate import odeint
import gc
from tqdm import tqdm
# np.seterr(divide='ignore', invalid='ignore')
import Simulator.coordinate as coord
import Simulator.environment as env
import Simulator.heating as heating


class Rocket:
    def __init__(self, json, json_engine):
        self.auto_end = json.get('Solver').get('Auto End Time')
        self.end_time = json.get('Solver').get('End Time [sec]')

        LP = json.get('Launch Pad')
        self.azimuth0 = LP.get('Launch Azimuth [deg]')
        self.elevation0 = LP.get('Launch Elevation [deg]')
        if self.elevation0 > 0.0:
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

        para = json.get('Parachute')
        self.timer_mode = para.get('Timer Mode')
        self.CdS1 = para.get('1st Parachute CdS [m2]')
        self.t_1st = para.get('1st Timer [s]')
        para2_exist = para.get('2nd Parachute Exist')
        if para2_exist:
            self.CdS2 = para.get('2nd Parachute CdS [m2]')
            self.alt_sepa2 = para.get('2nd Parachute Opening Altitude [m]')
            self.t_2nd_min = para.get('2nd Timer Min [s]')
            self.t_2nd_max = para.get('2nd Timer Max [s]')
        else:
            self.CdS2 = 0.0
            self.alt_sepa2 = 0.0
            self.t_2nd_min = 0.0
            self.t_2nd_max = 0.0

        aero = json.get('Aero')
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

        prop = json_engine.get('Propellant')
        self.m0_ox = prop.get('Oxidizer Mass [kg]')
        # self.mdot_ox = prop.get('Oxidizer Mass Flow Rate [kg/s]')
        self.m0_f = prop.get('Fuel Mass Before Burn [kg]')
        self.mf_after = prop.get('Fuel Mass After Burn [kg]')
        self.mdot_f = prop.get('Fuel Mass Flow Rate [kg/s]')
        self.d_out_f = prop.get('Fuel Outside Diameter [m]')
        self.d_port_f = prop.get('Fuel Inside Diameter [m]')
        self.L_f = prop.get('Fuel Length [m]')

        # Thrust, SL
        # 1st column : time [s]
        # 2nd column : thrust [N]
        engine = json_engine.get('Engine')
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
            self.total_impulse = round(np.sum(thrust_array) * np.sum(time_array) / len(time_array))
        else:
            time_array = np.arange(0.0, engine.get('Burn Time [sec]') + 0.01, 0.01)
            thrust_array = np.array([engine.get('Constant Thrust [N]')] * time_array.size)
            self.thrust = interpolate.interp1d(time_array, thrust_array, kind='linear', bounds_error=False, fill_value=(0.0, 0.0))
            self.total_impulse = round(engine.get('Constant Thrust [N]') * engine.get('Burn Time [sec]'))
        self.Isp = engine.get('Isp [sec]')
        self.dth = engine.get('Nozzle Throat Diameter [m]')
        self.eps = engine.get('Nozzle Expansion Ratio')
        self.Ath = 0.25 * np.pi * self.dth ** 2
        self.Ae = self.Ath * self.eps
        self.de = np.sqrt(self.Ae * 4.0 / np.pi)

        # Mass interpolate
        self.mdot_p_log = thrust_array / (self.Isp * 9.80665)
        self.mdot_p = interpolate.interp1d(time_array, self.mdot_p_log, kind='linear', bounds_error=False, fill_value=(0.0, 0.0))
        self.mdot_f = interpolate.interp1d(time_array, np.array([self.mdot_f]*len(time_array)), kind='linear', bounds_error=False, fill_value=(0.0, 0.0))
        self.mdot_ox_log = self.mdot_p_log - self.mdot_f(time_array)
        self.mdot_ox = interpolate.interp1d(time_array, self.mdot_ox_log, kind='linear', bounds_error=False, fill_value=(0.0, 0.0))

        mf_log_ode = odeint(lambda x, t: -self.mdot_f(t), self.m0_f, time_array)
        self.mf_log = mf_log_ode[0][0]
        for mf in mf_log_ode[1:]:
            self.mf_log = np.append(self.mf_log, mf[0])
        self.mf = interpolate.interp1d(time_array, self.mf_log, kind='linear', bounds_error=False, fill_value=(self.mf_log[0], self.mf_log[-1]))

        mox_log_ode = odeint(lambda x, t: -self.mdot_ox(t), self.m0_ox, time_array)
        self.mox_log = mox_log_ode[0][0]
        for mox in mox_log_ode[1:]:
            self.mox_log = np.append(self.mox_log, mox[0])
        self.mox = interpolate.interp1d(time_array, self.mox_log, kind='linear', bounds_error=False, fill_value=(self.mox_log[0], 0.0))

        self.Wind_refAltitude = json.get('Environment').get('Wind Mesurement Height [m]')
        self.Wind_power_exp = json.get('Environment').get('Wind Power Law Exponential Coefficient')

        self.m0_p = self.m0_ox + self.m0_f
        self.m0 = self.ms + self.m0_p
        self.Lcg0_p = (self.m0_ox * (self.L - self.Lcg0_ox) + self.m0_f * self.Lcg_f) / self.m0_p
        self.Lcg0 = (self.ms * self.Lcg_s + self.m0_p * self.Lcg0_p) / self.m0

        self.heat_obj = heating.NoseCone()
        heat_param = json.get('Aerodynamics Heating Parameter')
        self.heat_obj.T_surface_init = heat_param.get('Initial Surface Temperature [K]')
        self.heat_obj.R_nosetip = heat_param.get('Blunt Radius Tip [m]')
        self.heat_obj.thickness = heat_param.get('Thickness Tip [m]')
        self.heat_obj.rho = heat_param.get('Material Dencity [kg/m^3]')
        self.heat_obj.c = heat_param.get('Material Specific Heat [J/kg-K]')
        self.heat_obj.epsilon = heat_param.get('Matrial Surface Emissivity')
        self.heat_obj.T_ablation = heat_param.get('Ablation Temperature [K]')
        self.heat_obj.h_vaporization = heat_param.get('Vaporization Heat [kJ/kg]') / 1000.0


def dynamics(x, t, rocket, solver):
    Pos_ENU = x[0:3]
    altitude = Pos_ENU[2]
    Vel_ENU = x[3:6]
    Vel_Body = x[6:9]
    omega_Body = x[9:12]
    quat = x[12:16]
    # mf = x[16]
    mf = rocket.mf(t)
    # mox = x[17]
    mox = rocket.mox(t)
    quat = coord.quat_normalize(quat)
    on_launcher = altitude / np.sin(np.deg2rad(np.abs(rocket.elevation0))) < rocket.launcher_rail and t < 2.0

    # Translation coordinate
    DCM_ENU2Body = coord.DCM_ENU2Body_quat(quat)
    DCM_Body2ENU = DCM_ENU2Body.transpose()

    # Attitude
    # azimuth, elevation, roll = coord.quat2euler(DCM_ENU2Body)

    # alpha beta Vel_Air
    wind_ENU = env.Wind_ENU(solver.vel_wind, solver.angle_wind, altitude, rocket.Wind_refAltitude, rocket.Wind_power_exp)
    Vel_air = DCM_ENU2Body.dot(Vel_ENU - wind_ENU)
    Vel_air_abs = np.linalg.norm(Vel_air)
    u = Vel_air[0]
    v = Vel_air[1]
    w = Vel_air[2]

    if Vel_air_abs < 0.0 or np.abs(u) < 0.0 or on_launcher:
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

    mp = mf + mox
    m = rocket.ms + mp

    # Aero Force
    drag = dynamic_pressure * rocket.Cd(Mach) * rocket.A
    normal = dynamic_pressure * rocket.CNa(Mach) * rocket.A
    F_aero = np.array([-drag, normal * beta, -normal * alpha])

    # Newton Equation
    Force = (thrust + F_aero)
    Acc_Body =  Force / m + DCM_ENU2Body.dot(g)
    Acc_ENU = DCM_Body2ENU.dot(Force) / m + g

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
    Ij_roll = rocket.Ij0_s_roll + Ij_f_roll
    Ij = np.array([Ij_roll, Ij_pitch, Ij_pitch])
    Ijdot_f = np.array([Ij_f_roll * mdot_f / mf, Ij_f_pitch * mdot_f / mf, Ij_f_pitch * mdot_f / mf])

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

    if on_launcher:
        omegadot = np.array([0.0] * 3)
        quatdot = np.array([0.0] * 4)
        solver.vel_launch_clear = Vel_Body[0]
        solver.acc_launch_clear = Acc_Body[0]

    dx = np.zeros(16)
    dx[0:3] = Vel_ENU  # Pos_ENU
    dx[3:6] = Acc_ENU  # Vel_ENU
    dx[6:9] = Acc_Body  # Vel_Body
    dx[9:12] = omegadot  # omega_Body
    dx[12:16] = quatdot  # quat
    # dx[16] = -mdot_f  # mf
    # dx[17] = -mdot_ox  # mox

    return dx

def parachute_dynamics(x, t, rocket, solver):
    m = solver.m_apogee
    Pos_ENU = x[0:3]
    altitude = Pos_ENU[2]
    Vel_descent = x[3]

    wind_ENU = env.Wind_ENU(solver.vel_wind, solver.angle_wind, altitude, rocket.Wind_refAltitude, rocket.Wind_power_exp)
    Vel_ENU = np.array([wind_ENU[0], wind_ENU[1], Vel_descent])

    g = np.array([0.0, 0.0, -env.gravity(altitude)])
    Ta, Pa, rho, Cs = env.std_atmo(altitude)
    if rocket.timer_mode:
        if t > rocket.t_2nd_max or (altitude <= rocket.alt_sepa2 and t > rocket.t_2nd_min):
            CdS = rocket.CdS1 + rocket.CdS2
        else:
            CdS = rocket.CdS1
    else:
        CdS = rocket.CdS1 + rocket.CdS2 if altitude <= rocket.alt_sepa2 else rocket.CdS1
    drag = 0.5 * rho * CdS * Vel_ENU[2] ** 2
    Acc = drag / m + g[2]
    dx = np.zeros(4)
    dx[0:3] = Vel_ENU  # Pos_ENU
    dx[3] = Acc  # Vel_descent

    return dx

class Solver:
    def __init__(self, vel_wind, angle_wind, result_dir, multi_mode=False):
        self.vel_wind = vel_wind
        self.angle_wind = angle_wind
        self.result_dir = result_dir
        self.multi_mode = multi_mode

        # ランチクリア値、計算中に記録
        self.acc_launch_clear = 0.0
        self.vel_launch_clear = 0.0

    def solve(self, rocket):
        length = (rocket.L - rocket.Lcg0) * np.cos(np.deg2rad(rocket.elevation0))
        X0_ENU = length * np.cos(np.deg2rad(rocket.azimuth0))
        Y0_ENU = length * np.sin(np.deg2rad(rocket.azimuth0))
        Z0_ENU = (rocket.L - rocket.Lcg0) * np.sin(np.deg2rad(np.abs(rocket.elevation0)))
        Pos0_ENU = np.array([X0_ENU, Y0_ENU, Z0_ENU])
        quat0 = coord.euler2quat(rocket.azimuth0, rocket.elevation0, rocket.roll0)
        zero_array = np.array([[0.0] * 3])

        x0 = np.zeros(16)
        x0[0:3] = Pos0_ENU
        x0[3:6] = zero_array  # Vel_ENU
        x0[6:9] = zero_array  # Vel_Body
        x0[9:12] =  zero_array  # omega_Body
        x0[12:16] = quat0  # quat
        # x0[16] = rocket.m0_f  # mf
        # x0[17] = rocket.m0_ox  # mox
        start_time = 0.0
        def estimate_end():
            It_digits = len(str(int(rocket.total_impulse)))
            return rocket.total_impulse / (10 ** (It_digits - 3))
        end_time = estimate_end() if rocket.auto_end else rocket.end_time
        def dicide_timestep(end_time):
            time = int(end_time)
            digit = len(str(time))
            digit += round(int(str(time)[0])) // 10
            return 0.00001 * 10.0 ** digit
        time_step = dicide_timestep(end_time)
        time = np.arange(start_time, end_time, time_step)
        ode_log = odeint(dynamics, x0, time, args=(rocket, self))

        self.time_log = time
        self.Pos_ENU_log = ode_log[:, 0:3]
        self.Vel_ENU_log = ode_log[:, 3:6]
        self.Vel_Body_log = ode_log[:, 6:9]
        self.omega_log = ode_log[:, 9:12]
        self.quat_log = ode_log[:, 12:16]
        # self.mf_log = ode_log[:, 16]
        self.mf_log = rocket.mf(self.time_log)
        # self.mox_log = ode_log[:, 17]
        self.mox_log = rocket.mox(self.time_log)
        self.post_process(rocket)

    def post_process(self, rocket):
        # 着地後の分をカット
        index_hard_landing = np.argmax(self.Pos_ENU_log[:, 2] <= 0.0)
        time_log = self.time_log[:index_hard_landing+1]
        Pos_ENU_log = self.Pos_ENU_log[:index_hard_landing+1, :]
        Vel_ENU_log = self.Vel_ENU_log[:index_hard_landing+1, :]
        Vel_Body_log = self.Vel_Body_log[:index_hard_landing+1, :]
        omega_log = self.omega_log[:index_hard_landing+1, :]
        quat_log = self.quat_log[:index_hard_landing+1, :]
        quat_log = np.array(list(map(coord.quat_normalize, quat_log)))
        mf_log = self.mf_log[:index_hard_landing+1]
        mox_log = self.mox_log[:index_hard_landing+1]

        # あとから計算
        altitude_log = Pos_ENU_log[:, 2]
        downrange_log = np.array(list(map(np.linalg.norm, Pos_ENU_log[:, 0:2])))
        g_log = np.array(list(map(env.gravity, altitude_log)))
        P_air_log = env.get_std_press_array(altitude_log)
        rho_air_log = env.get_std_density_array(altitude_log)
        mp_log = mox_log + mf_log
        m_log = mp_log + rocket.ms

        DCM_ENU2Body_log = np.array(list(map(coord.DCM_ENU2Body_quat, quat_log)))
        wind_ENU_log = np.array([env.Wind_ENU(self.vel_wind, self.angle_wind, alt, rocket.Wind_refAltitude, rocket.Wind_power_exp) for alt in altitude_log])
        Vel_air_log = np.array([DCM.dot(vel - wind) for DCM, vel, wind in zip(DCM_ENU2Body_log, Vel_ENU_log, wind_ENU_log)])
        Vel_air_abs_log = np.array(list(map(np.linalg.norm, Vel_air_log)))
        on_launcher = (altitude_log / np.sin(np.deg2rad(np.abs(rocket.elevation0)))) > rocket.launcher_rail
        alpha_log = np.rad2deg(np.arctan2(Vel_air_log[:, 2], Vel_air_log[:, 0])) * on_launcher
        # alpha_log = np.array([np.rad2deg(np.arctan2(w, u)) for w, u in zip(Vel_air_log[:, 2], Vel_air_log[:, 0])])
        beta_log = np.rad2deg(np.arcsin(-Vel_air_log[:, 1] / Vel_air_abs_log)) * on_launcher
        # beta_log = np.array([np.rad2deg(np.arcsin(-v / vel)) for v, vel in zip(Vel_air_log[:, 1], Vel_air_abs_log)])
        Mach_log = Vel_air_abs_log / env.get_std_soundspeed_array(altitude_log)
        dynamic_pressure_log = 0.5 * rho_air_log * Vel_air_abs_log * Vel_air_abs_log
        attitude_log = np.array(list(map(coord.quat2euler, DCM_ENU2Body_log)))
        azimuth_log = attitude_log[:, 0]
        elevation_log = attitude_log[:, 1]
        roll_log = attitude_log[:, 2]
        thrust_SL_log = rocket.thrust(time_log)
        # mdot_p_log = thrust_SL_log / (rocket.Isp * 9.80665)
        mdot_p_log = rocket.mdot_p(time_log)
        # mdot_f_log = rocket.mdot_f * np.array([thrust > 0.0 for thrust in rocket.thrust(time_log)])
        mdot_f_log = rocket.mdot_f(time_log)
        # mdot_ox_log = mdot_p_log - mdot_f_log
        mdot_ox_log = rocket.mdot_ox(time_log)
        pressure_thrust_log = (P_air_log[0] - P_air_log) * rocket.Ae
        thrust_log = np.array([thrust_SL + pressure_thrust if thrust_SL > 0.0 else 0.0 for thrust_SL, pressure_thrust in zip(thrust_SL_log, pressure_thrust_log)])
        Isp_log = rocket.Isp + np.array([pressure_thrust / (mdot_p * 9.80665) if mdot_p > 0.0 else 0.0 for pressure_thrust, mdot_p in zip(pressure_thrust_log, mdot_p_log)])
        drag_log = dynamic_pressure_log * rocket.Cd(Mach_log) * rocket.A
        normal_log = dynamic_pressure_log * rocket.CNa(Mach_log) * rocket.A
        Force_log = np.c_[thrust_log - drag_log, normal_log * np.deg2rad(beta_log), -normal_log * np.deg2rad(alpha_log)]
        Acc_Body_log = np.array([Force / m + DCM.dot(g) for Force, m, DCM, g in zip(Force_log, m_log, DCM_ENU2Body_log, np.c_[np.zeros_like(g_log), np.zeros_like(g_log), -g_log])])
        Lcg_ox_log = rocket.L - (rocket.L_motor + (mox_log / rocket.m0_ox) * (rocket.Lcg0_ox - rocket.L_motor))
        Lcg_p_log = (mf_log * rocket.Lcg_f + mox_log * Lcg_ox_log) / mp_log
        Lcg_log = (mp_log * Lcg_p_log + rocket.ms * rocket.Lcg_s) / m_log
        Lcp_log = rocket.Lcp(Mach_log)
        Fst_log = (Lcp_log - Lcg_log) / rocket.L * 100  # [%]

        heater = heating.FlightHeating(time_log, Vel_air_abs_log, altitude_log)
        heater.heating(rocket.heat_obj)
        q_conv_log = heater.q_conv
        q_rad_log = heater.q_rad
        q_heat_log = q_conv_log + q_rad_log
        T_surface_log = heater.T_surface
        thickness_log = heater.thickness

        #  イベントでの値
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
        self.m_apogee = m_log[index_apogee]
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

        #  パラシュート降下
        x0 = np.zeros(4)
        x0[0:3] = self.Pos_ENU_apogee
        x0[3] = self.Vel_ENU_apogee[2]
        est_landing = self.altitude_apogee / np.sqrt(2.0 * self.m_apogee * env.gravity(0.0) / (env.get_std_density(0.0) * (rocket.CdS1 + rocket.CdS2)))
        time = np.arange(self.time_apogee, self.time_hard_landing+est_landing, 0.1)
        ode_log = odeint(parachute_dynamics, x0, time, args=(rocket, self))

        dV_decent = (ode_log[:-1, 3] - ode_log[1:, 3]) / 0.1
        index_sepa2 = np.argmin(dV_decent)
        # index_sepa2 = np.argmax(ode_log[:, 2] <= rocket.alt_sepa2)
        index_soft_landing = np.argmax(ode_log[:, 2] <= 0.0)
        self.time_sepa2 = time[index_sepa2]
        self.time_soft_landing = time[index_soft_landing]
        self.soft_landing_point = ode_log[index_soft_landing, 0:2]
        self.downrange_soft_landing = np.linalg.norm(self.soft_landing_point)
        self.Vel_descent_1st = ode_log[index_sepa2-1, 3]
        self.Vel_descent_2nd = ode_log[index_soft_landing, 3]

        if self.multi_mode:
            return

        txt = open(self.result_dir + '/result.txt', mode='w')
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
        txt.writelines(['Max Tip Temperature,', str(round(max(T_surface_log), 2)), '[K]\n'])
        txt.writelines(['Terminal Tip Thickness,', str(round(thickness_log[-1], 2)), '[mm]\n'])
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
            q_conv_log,
            q_rad_log,
            q_heat_log,
            T_surface_log,
            thickness_log,
            Acc_Body_log,
            Vel_Body_log,
            Vel_ENU_log,
            Pos_ENU_log,
        ]
        header = 'time,g,P_air,rho_air,wind_East,wind_North,wind_Up,mdot_f,mdot_ox,mdot_p,mf,mox,mp,m,Lcg_ox,Lcg_p,Lcg,Lcp,Fst,alpha,beta,omega_roll,omega_pitch,omega_yaw,quat0,quat1,quat2,quat3,azimuth,elevation,roll,thrust,Isp,drag,normal,Force_X,Force_Y,Force_Z,Vel_air_X,Vel_air_Y,Vel_air_Z,Vel_air_abs,Mach,dynamic_pressure,q_conv,q_rad,q_heat,T_surface,tip_thickness,Acc_Body_X,Acc_Body_Y,Acc_Body_Z,Vel_Body_X,Vel_Body_Y,Vel_Body_Z,Vel_East,Vel_North,Vel_Up,Pos_East,Pos_North,Pos_Up'
        np.savetxt(self.result_dir + '/log.csv', output_array, delimiter=',', header=header)

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

        plt.figure('Aerodynamics Heating')
        plt.plot(time_log, q_conv_log / 10**6, label='convection heat')
        plt.plot(time_log, q_rad_log / 10**6, label='radiation heat')
        plt.plot(time_log, q_heat_log / 10**6, label='total heat')
        plt.xlabel('Time [sec]')
        plt.ylabel('q dot [MW/m^2]')
        plt.xlim(xmin=0.0)
        plt.grid()
        plt.legend()
        plt.savefig(self.result_dir + '/AerodynamicsHeating.png')

        plt.figure('Surface Temperature')
        plt.plot(time_log, T_surface_log, label='Surface Temperature')
        plt.xlabel('Time [sec]')
        plt.ylabel('Surface Temperature [K]')
        plt.xlim(xmin=0.0)
        plt.grid()
        plt.legend()
        plt.savefig(self.result_dir + '/SurfaceTemperature.png')

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

        plt.figure('Body Velocity')
        plt.plot(time_log, Vel_Body_log[:, 0], label='V_Body X')
        plt.plot(time_log, Vel_Body_log[:, 1], label='V_Body Y')
        plt.plot(time_log, Vel_Body_log[:, 2], label='V_Body Z')
        plt.xlabel('Time [sec]')
        plt.ylabel('Body Velocity [m/s]')
        plt.xlim(xmin=0.0)
        plt.grid()
        plt.legend()
        plt.savefig(self.result_dir + '/VelocityBody.png')

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
        plt.plot(time[:index_soft_landing], ode_log[:index_soft_landing,3])
        plt.xlabel('Time [sec]')
        plt.ylabel('Velocity [m/s]')
        plt.xlim(xmin=self.time_apogee)
        plt.grid()
        plt.legend()
        plt.savefig(self.result_dir + '/DecentVel.png')

        plt.figure('Decent Altitude')
        plt.plot(time[:index_soft_landing], ode_log[:index_soft_landing,2])
        plt.xlabel('Time [sec]')
        plt.ylabel('Altitude [m]')
        plt.xlim(xmin=self.time_apogee)
        plt.grid()
        plt.legend()
        plt.savefig(self.result_dir + '/DecentAlt.png')

class Mapper4Wind:
    def __init__(self, vel_wind_config, angle_wind_config, result_dir):
        self.result_dir = result_dir
        Vel_wind_min = vel_wind_config[0]
        Vel_wind_max = vel_wind_config[1]
        Vel_wind_step = vel_wind_config[2]
        angle_wind_min = angle_wind_config[0]
        angle_wind_max = angle_wind_config[1]
        angle_wind_step = angle_wind_config[2]

        self.Vel_wind_array = np.arange(Vel_wind_min, Vel_wind_max + Vel_wind_step, Vel_wind_step)
        self.angle_wind_array = np.arange(angle_wind_min, angle_wind_max + angle_wind_step, angle_wind_step)

    def mapping(self, rocket):
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
                solver = Solver(Vel_wind, angle_wind, self.result_dir, True)
                solver.solve(rocket)
                # Result Pickup
                time_apogee_array_angle = np.append(time_apogee_array_angle, solver.time_apogee)
                Vel_air_apogee_array_angle = np.append(Vel_air_apogee_array_angle, solver.Vel_air_abs_apogee)
                altitude_apogee_array_angle = np.append(altitude_apogee_array_angle, solver.altitude_apogee)
                downrange_hard_array_angle = np.append(downrange_hard_array_angle, solver.downrange_hard_landing)
                downrange_soft_array_angle = np.append(downrange_soft_array_angle, solver.downrange_soft_landing)
                Vel_air_max_array_angle = np.append(Vel_air_max_array_angle, solver.Vel_air_max)
                Mach_max_array_angle = np.append(Mach_max_array_angle, solver.Mach_max)
                MaxQ_array_angle = np.append(MaxQ_array_angle, solver.maxQ)
                time_hard_landing_array_angle = np.append(time_hard_landing_array_angle, solver.time_hard_landing)
                hard_landing_points_angle.append(solver.hard_landing_point)
                time_sepa2_array_angle = np.append(time_sepa2_array_angle, solver.time_sepa2)
                time_soft_landing_array_angle = np.append(time_soft_landing_array_angle, solver.time_soft_landing)
                soft_landing_points_angle.append(solver.soft_landing_point)
                del solver
                gc.collect()
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

        np.savetxt(self.result_dir + '/time_apogee.csv', time_apogee_array, delimiter=',')
        np.savetxt(self.result_dir + '/vel_apogee.csv', Vel_air_apogee_array, delimiter=',')
        np.savetxt(self.result_dir + '/altitude_apogee.csv', altitude_apogee_array, delimiter=',')
        np.savetxt(self.result_dir + '/downrange_hard.csv', downrange_hard_array, delimiter=',')
        np.savetxt(self.result_dir + '/downrange_soft.csv', downrange_soft_array, delimiter=',')
        np.savetxt(self.result_dir + '/vel_max.csv', Vel_air_max_array, delimiter=',')
        np.savetxt(self.result_dir + '/mach_max.csv', Mach_max_array, delimiter=',')
        np.savetxt(self.result_dir + '/maxQ.csv', MaxQ_array, delimiter=',')
        np.savetxt(self.result_dir + '/time_hard_landing.csv', time_hard_landing_array, delimiter=',')
        np.savetxt(self.result_dir + '/time_sepa_2nd.csv', time_sepa2_array, delimiter=',')
        np.savetxt(self.result_dir + '/time_soft_landing.csv', time_soft_landing_array, delimiter=',')

        return self.Vel_wind_array, self.angle_wind_array, hard_landing_points, soft_landing_points
