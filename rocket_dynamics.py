# -*- coding: utf-8 -*-
import sys
import json
import numpy as np
from scipy import interpolate
from scipy.integrate import odeint

import coordinate as coord
import environment as env

##################################
# Config Class
# Hold constant value of Rocket
##################################
class Config:  
  def __init__(self, json):
    # Time Parameter
    self.StartTime = json.get('Start Time [sec]')
    self.EndTime = json.get('End Time [sec]')
    self.TimeStep = json.get('Time Step [sec]')

    self.LaunchLLH = np.array(json.get('Launch Pad').get('LLH [deg, deg, m]'))
    self.LaunchElevation = json.get('Launch Pad').get('Launch Elevation [deg]')
    self.LaunchAzimuth = json.get('Launch Pad').get('Launch Azimuth [deg]')
    self.LaunchRoll = json.get('Launch Pad').get('Launch Roll Angle [deg]')
    self.LRail = json.get('Launch Pad').get('Launcher Rail Length [m]')

    self.L = json.get('Structure').get('Length [m]')
    self.d = json.get('Structure').get('Diameter [m]')
    self.A = 0.25 * np.pi * self.d * self.d
    self.Ms = json.get('Structure').get('Structure Mass [kg]')
    self.Lcgs = json.get('Structure').get('Structure Length-C.G. FormNoseTip [m]')
    self.Lcgox0 = json.get('Structure').get('Initial Oxidizer Length-C.G. FromEnd [m]') # FromEndからNoseTipへの変換は計算中で
    Lcgf_FromEnd = json.get('Structure').get('Fuel Length-C.G. FromEnd [m]')
    self.Lcgf = self.L - Lcgf_FromEnd
    self.L_engine = json.get('Structure').get('Length End-to-Tank [m]')
    self.Ijs = np.zeros(3)
    self.Ijs[0] = json.get('Structure').get('Structure Inertia-Moment Roll-Axis [kg*m^2]')
    self.Ijs[1] = json.get('Structure').get('Structure Inertia-Moment Pitch-Axis [kg*m^2]')
    self.Ijs[2] = self.Ijs[1]

    # Drag Coefficient
    # 1st column : Mach
    # 2nd column : Cd
    CdFile = json.get('Aero').get('Cd File')
    CdFile_exist = json.get('Aero').get('File Exist')    
    if CdFile_exist:
      Cd_Base = np.loadtxt(CdFile, delimiter=",", skiprows=1)
      self.Cd = interpolate.interp1d(Cd_Base[:,0], Cd_Base[:,1], bounds_error=False, fill_value="extrapolate")
    else:
      Mach_Base = np.arange(0.0, 20.5, 0.5)
      Cd_Base = np.array([[json.get('Aero').get('Constant Cd')] * Mach_Base.size])
      self.Cd = interpolate.interp1d(Mach_Base, Cd_Base, bounds_error=False, fill_value="extrapolate")
    self.Lcp = json.get('Aero').get('Length-C.P. [m]')
    self.CNa = json.get('Aero').get('Normal Coefficient CNa')
    self.Clp = json.get('Aero').get('Roll Dumping Moment Coefficient Clp')
    self.Cmq = json.get('Aero').get('Pitch Dumping Moment Coefficient Cmq')
    self.Cnr = self.Cmq
   

    self.Mox0 = json.get('Propellant').get('Oxidizer Mass [kg]')
    self.Mdot_ox = json.get('Propellant').get('Oxidizer Mass Flow Rate [kg/s]')
    self.Mf_before = json.get('Propellant').get('Fuel Mass Before Burn [kg]')
    self.Mf_after = json.get('Propellant').get('Fuel Mass After Burn [kg]')
    self.Mdot_f = json.get('Propellant').get('Fuel Mass Flow Rate [kg/s]')
    self.Df_out = json.get('Propellant').get('Fuel Outside Diameter [m]')
    self.Df_port = json.get('Propellant').get('Fuel Inside Diameter [m]')
    self.Lf = json.get('Propellant').get('Fuel Length [m]')
    self.TEKtank = json.get('Propellant').get('Use HyperTEK Tank')
    self.refMdot_p = self.Mdot_ox + self.Mdot_f # Use Isp change

    # Thrust, SL
    # 1st column : time [s]
    # 2nd column : thrust [N]
    ThrustFile = json.get('Engine').get('Thrust File')
    ThrustFile_exist = json.get('Engine').get('File Exist')
    if ThrustFile_exist:
      Thrust_Base = np.loadtxt(ThrustFile, delimiter=",", skiprows=1)
      def ThrustCut(Thrust_Original, M):
        Time = Thrust_Original[:,0]
        Thrust = Thrust_Original[:,1]
        th = M * 9.80665
        index = (Thrust < th).argmin()
        Thrust = Thrust[index:]
        Time = Time[:len(Thrust)]
        return Time, Thrust
      Time_Base, Thrust_Base = ThrustCut(Thrust_Base, self.Ms + self.Mf_before + self.Mox0)
      self.thrust = interpolate.interp1d(Time_Base, Thrust_Base, bounds_error=False, fill_value=(0.0, 0.0))
    else:    
      Time_Base = np.arange(0.0, json.get('Engine').get('Burn Time [sec]') + 0.1, 0.1)
      Thrust_Base = np.array([[json.get('Engine').get('Constant Thrust [N]')] * Time_Base.size])
      self.thrust = interpolate.interp1d(Time_Base, Thrust_Base, bounds_error=False, fill_value=(0.0, 0.0))
      
    self.Isp = json.get('Engine').get('Isp [sec]')
    dth = json.get('Engine').get('Nozzle Throat Diameter [m]')
    ExpansionR = json.get('Engine').get('Nozzle Expansion Ratio')
    Ath = 0.25 * np.pi * dth * dth
    self.Ae = Ath * ExpansionR
    self.de = np.sqrt(4.0 * self.Ae / np.pi)
    
    self.Wind_refAltitude = json.get('Environment').get('Wind Mesurement Height [m]')
    self.Wind_power_exp = json.get('Environment').get('Wind Power Law Exponential Coefficient')


##################################
# def Rocket Dynamics
# Call this for Simulation
##################################
def Simulation(rocket, WindSpeed, WindDirection, ResultDir, result):
  # Center of Gravity
  # Lcgox0 => pre calculation other script
  def center_gravity(rocket, Mox, Mf):
    Lcgox = rocket.L - (rocket.L_engine + (Mox / rocket.Mox0) * (rocket.Lcgox0 - rocket.L_engine))
    Lcgp = (Mf * rocket.Lcgf + Mox * Lcgox) / (Mf + Mox)
    Lcg = ((Mf + Mox) * Lcgp + rocket.Ms * rocket.Lcgs) / ((Mf + Mox) + rocket.Ms)
    return Lcg, Lcgp

  def inertia_moment(rocket, Mf, Lcg):
    # Fuel Inertia Moment
    pitch = Mf * ((rocket.Df_port ** 2 + rocket.Df_out ** 2) / 4.0 + rocket.Lf / 3.0) / 4.0
    roll = Mf * (rocket.Df_port ** 2 + rocket.Df_out ** 2) / 8.0
    Ijf = np.array([roll, pitch, pitch])

    # Rotation Axis Offset
    pitchs = rocket.Ijs[1] + rocket.Ms * (Lcg - rocket.Lcgs) ** 2 # Structure
    pitchf = Ijf[1] + Mf * (Lcg - rocket.Lcgf) ** 2 # Fuel
    roll = Ijf[0] + rocket.Ijs[0]

    Ij = np.array([roll, pitchf + pitchs, pitchf + pitchs])

    # Use Dumping Moment : param Mf => Mdot_f 
    Ijf_dot = np.array([roll - rocket.Ijs[0], pitchf, pitchf])

    return Ij, Ijf_dot
  
  def Angle_of_Attack(Vel_Air_Bodyframe):
    Vel_Air_abs = np.linalg.norm(Vel_Air_Bodyframe)
    u = Vel_Air_Bodyframe[0]
    v = Vel_Air_Bodyframe[1]
    w = Vel_Air_Bodyframe[2]

    alpha = np.arctan2(w, u)
    beta = np.arctan2(v, Vel_Air_abs)

    return alpha, beta

  def AeroForce(rocket, DynamicPressure, alpha, beta, Mach):
    Drag = DynamicPressure * rocket.Cd(Mach) * rocket.A
    Normal = DynamicPressure * rocket.CNa * rocket.A
    return np.array([-Drag, Normal * beta, -Normal * alpha])

  def AeroMoment(rocket, AeroForce, Lcg):
    return np.array([0.0, (rocket.Lcp - Lcg) * AeroForce[2], -(rocket.Lcp - Lcg) * AeroForce[1]])
  def AeroDumpingMoment(rocket, DynamicPressure, Vel_Air_abs, Omega_Body):
    Ka = np.zeros(3)
    Ka[0] = DynamicPressure * rocket.Clp * rocket.A * rocket.d * rocket.d * 0.5 / Vel_Air_abs * Omega_Body[0]
    Ka[1] = DynamicPressure * rocket.Cmq * rocket.A * rocket.L * rocket.L * 0.5 / Vel_Air_abs * Omega_Body[1]
    Ka[2] = DynamicPressure * rocket.Cnr * rocket.A * rocket.L * rocket.L * 0.5 / Vel_Air_abs * Omega_Body[2]
    return np.array([Ka[0], Ka[1], Ka[2]])
  def JetDumpingMoment(rocket, Lcg, Lcgp, Mdot_p, Omega_Body):
    _, Ijf_dot = inertia_moment(rocket, rocket.Mdot_f, Lcg)
    Kj = np.zeros(3)
    Kj[0] = -(Ijf_dot[0] - rocket.Mdot_f * 0.5 * (0.25 * rocket.de * rocket.de)) * Omega_Body[0]
    Kj[1] = -(Ijf_dot[1] + Mdot_p * ((Lcg - Lcgp) ** 2 - (rocket.L - Lcgp) ** 2)) * Omega_Body[1]
    Kj[1] = -(Ijf_dot[2] + Mdot_p * ((Lcg - Lcgp) ** 2 - (rocket.L - Lcgp) ** 2)) * Omega_Body[2]
    return np.array([Kj[0], Kj[1], Kj[2]])

  def EulerEquation(Ij, Omega_Body, Moment):
    p = Omega_Body[0]
    q = Omega_Body[1]
    r = Omega_Body[2]
    dpdt = ((Ij[1] - Ij[2]) * q * r + Moment[0]) / Ij[0]
    dqdt = ((Ij[2] - Ij[0]) * p * r + Moment[1]) / Ij[1]
    drdt = ((Ij[0] - Ij[1]) * p * q + Moment[2]) / Ij[2]

    return np.array([dpdt, dqdt, drdt])

  def KinematicEquation(Quat, Omega_Body):
    p = Omega_Body[0]
    q = Omega_Body[1]
    r = Omega_Body[2]
    tensor = np.array([[0.0, -p , -q , -r ],
                       [p  , 0.0, r  , -q ],
                       [q  , -r , 0.0, p  ],
                       [r  , q  , -p , 0.0]])
    return 0.5 * tensor.dot(Quat)
  
  # Differential Equations collection For odeint
  def dynamics(x, t, params, rocket, retult):
    WindSpeed = params[0]
    WindDirection = params[1]
    Mf = x[0]
    Mox = x[1]
    # Limit Negative Propellant Mass
    if Mf <= 0.0:
      Mf = 0.0
    if Mox <= 0.0:
      Mox = 0.0

    Vel_ECI = np.array([x[2], x[3], x[4]])
    Pos_ECI = np.array([x[5], x[6], x[7]])
    Omega_Body = np.array([x[8], x[9], x[10]])
    Quat = np.array([x[11], x[12], x[13], x[14]])
    Quat = coord.Quat_normalize(Quat)

    # Transformation Coordinate
    # Position
    DCM_ECI2ECEF = coord.DCM_ECI2ECEF(t)
    Pos_ECEF = DCM_ECI2ECEF.dot(Pos_ECI)
    Pos_LLH = coord.ECEF2LLH(Pos_ECEF)
    Latitude = Pos_LLH[0]
    Longitude = Pos_LLH[1]
    Altitude = Pos_LLH[2]
    # Azimuth Elevation
    DCM_ECEF2NED = coord.DCM_ECEF2NED(Latitude, Longitude)
    DCM_NED2ECEF = DCM_ECEF2NED.transpose()
    DCM_ECI2NED = DCM_ECEF2NED.dot(DCM_ECI2ECEF)
    DCM_NED2ECI = DCM_ECI2NED.transpose()
    DCM_NED2Body_Quat = coord.DCM_NED2Body_Quat(Quat)
    Azimuth, Elevation, Roll = coord.Quat2Euler(DCM_NED2Body_Quat)
    DCM_Body2ECI = DCM_NED2Body_Quat.dot(DCM_NED2ECI)
    # alpha beta Vel_Air
    Wind_NED = env.Wind_NED(WindSpeed, WindDirection, Altitude, rocket.Wind_refAltitude, rocket.Wind_power_exp)
    Vel_ECEF_NEDframe = coord.Vector_ECI2ECEF_NEDframe(DCM_ECI2NED, Vel_ECI, Pos_ECI)  
    Vel_ECEF_Bodyframe = DCM_NED2Body_Quat.dot(Vel_ECEF_NEDframe)    
    Vel_Air_Bodyframe = DCM_NED2Body_Quat.dot(Vel_ECEF_NEDframe - Wind_NED)
    alpha, beta = Angle_of_Attack(Vel_Air_Bodyframe)
    DCM_Body2Air = coord.DCM_Body2Air(alpha, beta)
    DCM_Air2Body = DCM_Body2Air.transpose()
   
    # Air Environment
    g0 = 9.80665
    g = np.array([0.0, 0.0, -env.gravity(Altitude)])
    Ta0, Pa0, rho0, Cs0 = env.std_atmo(0.0)
    Ta, Pa, rho, Cs = env.std_atmo(Altitude)
    Mach = np.linalg.norm(Vel_Air_Bodyframe) / Cs
    DynamicPressure = 0.5 * rho * np.linalg.norm(Vel_Air_Bodyframe) ** 2
    
    # Transration
    Mp = Mf + Mox
    M = rocket.Ms + Mp
    if rocket.thrust(t) > 0.0:
      thrust = np.array([rocket.thrust(t) + (Pa0 - Pa) * rocket.Ae, 0.0, 0.0])      
      Isp = rocket.Isp + (Pa0 - Pa) * rocket.Ae / (rocket.refMdot_p * g0)      
      Mdot_p = thrust[0] / (Isp * g0)
      Mdot_f = rocket.Mdot_f
      Mdot_ox = Mdot_p - Mdot_f
      # Limit Increase and Empty Oxidizers
      if Mdot_ox < 0.0 or Mox <= 0.0:
        Mdot_ox = 0.0
    else:
      thrust = np.zeros(3)      
      Isp = 0.0
      Mdot_p = 0.0
      Mdot_f = 0.0
      Mdot_ox = 0.0
    Aeroforce = AeroForce(rocket, DynamicPressure, alpha, beta, Mach)
    # Newton Equation
    Acc_ECI = DCM_Body2ECI.dot(thrust + DCM_Air2Body.dot(Aeroforce)) / M + DCM_NED2ECI.dot(g)
    # Add Coriolis Force
    Acc_ECI[0] = Omega_Body[2] * Vel_ECEF_Bodyframe[1] - Omega_Body[1] * Vel_ECEF_Bodyframe[2] + Acc_ECI[0]
    Acc_ECI[1] = Omega_Body[0] * Vel_ECEF_Bodyframe[2] - Omega_Body[2] * Vel_ECEF_Bodyframe[0] + Acc_ECI[1]
    Acc_ECI[2] = Omega_Body[1] * Vel_ECEF_Bodyframe[0] - Omega_Body[0] * Vel_ECEF_Bodyframe[1] + Acc_ECI[2]

    # Rotation
    Lcg, Lcgp = center_gravity(rocket, Mox, Mf)
    Ij, _ = inertia_moment(rocket, Mf, Lcg)
    Aero_moment = AeroMoment(rocket, Aeroforce, Lcg)    
    Aero_dumping_moment = AeroDumpingMoment(rocket, DynamicPressure, np.linalg.norm(Vel_Air_Bodyframe), Omega_Body)
    Jet_dumping_moment = JetDumpingMoment(rocket, Lcg, Lcgp, Mdot_p, Omega_Body)
    Moment = DCM_Air2Body.dot(Aero_moment + Aero_dumping_moment + Jet_dumping_moment)
    dOmega_Body = EulerEquation(Ij, Omega_Body, Moment)
    dQuat = KinematicEquation(Quat, Omega_Body)

    dx = np.zeros(15)
    dx[0] = -rocket.Mdot_f # dMf/dt
    dx[1] = -Mdot_ox # dMox/dt
    dx[2] = Acc_ECI[0] # dVx_ECI/dt
    dx[3] = Acc_ECI[1] # dVy_ECI/dt
    dx[4] = Acc_ECI[2] # dVz_ECI/dt
    dx[5] = Vel_ECI[0] # dX_ECI/dt
    dx[6] = Vel_ECI[1] # dY_ECI/dt
    dx[7] = Vel_ECI[2] # dZ_ECI/dt
    dx[8] = dOmega_Body[0] # dOmegax_Body/dt
    dx[9] = dOmega_Body[1] # dOmegay_Body/dt
    dx[10] = dOmega_Body[2] # dOmegaz_Body/dt
    dx[11] = dQuat[0] # dq0/dt
    dx[12] = dQuat[1] # dq1/dt
    dx[13] = dQuat[2] # dq2/dt
    dx[14] = dQuat[3] # dq3/dt

    # 着地後そのまま計算を続けると高度の関数になっているものあたり(重力かな？)が影響してバウンス、タイムステップが小さくなってソルバが停止するのでとりあえず対処
    if Altitude < 0.0:
      dx[2] = 0.0
      dx[3] = 0.0
      dx[4] = 0.0
    # 着地まで結果を出力
    if Altitude < 0.0:
      output_items = np.array([t, Mf, Mox, M, Mdot_ox, Mdot_f, Lcgp, Lcg, thrust[0], Isp, Aeroforce[0], Aeroforce[1], Aeroforce[2],
                              Acc_ECI[0], Acc_ECI[1], Acc_ECI[2], Vel_ECI[0], Vel_ECI[1], Vel_ECI[2], Pos_ECI[0], Pos_ECI[1], Pos_ECI[2], 
                              Pos_ECEF[0], Pos_ECEF[1], Pos_ECEF[2], Pos_LLH[0], Pos_LLH[1], Pos_LLH[2], Azimuth, Elevation,
                              Wind_NED[0], Wind_NED[1], Wind_NED[2], Vel_Air_Boodyframe[0], Vel_Air_Boodyframe[1], Vel_Air_Boodyframe[2], alpha, beta, Mach, DynamicPressure, g, Pa, rho,
                              Ij[0], Ij[1], Ij[2], Moment[0], Moment[1], Moment[2], Omega[0], Omega[1], Omega[2]])
      result.value = np.vstack((result.value, output_items))
      

    # odeintの時間長くすると動作が見えないのでプログレスバーを表示
    sys.stderr.write ('\r\033[K' + '[' + '=' * int((t / rocket.EndTime) * 50) + '>' + ' ' * (50 - int((t / rocket.EndTime) * 50)) + ']' + ' ' + '%0.1f/%0.1f sec %0.2f' %(t, rocket.EndTime, Altitude))
    sys.stderr.flush()

    return dx

  def Initialize(rocket):    
    DCM_ECI2ECEF = coord.DCM_ECI2ECEF(0.0)
    DCM_ECEF2ECI = DCM_ECI2ECEF.transpose()
    Pos_ECEF = coord.LLH2ECEF(rocket.LaunchLLH)   
    Pos_ECI = DCM_ECEF2ECI.dot(Pos_ECEF)
    Vel_ECEF_NEDframe = np.array([0.0, 0.0, 0.0])
    DCM_ECEF2NED = coord.DCM_ECEF2NED(rocket.LaunchLLH[0], rocket.LaunchLLH[1])
    DCM_ECI2NED = DCM_ECEF2ECI.dot(DCM_ECEF2NED)
    DCM_NED2ECI = DCM_ECI2NED.transpose()
    Vel_ECI = coord.Vector_NED2ECI(DCM_NED2ECI, Vel_ECEF_NEDframe, Pos_ECI)

    Quat = coord.Euler2Quat(rocket.LaunchAzimuth, rocket.LaunchElevation, rocket.LaunchRoll)

    x0 = np.zeros(15)
    x0[0] = rocket.Mf_before
    x0[1] = rocket.Mox0
    x0[2] = Vel_ECI[0]
    x0[3] = Vel_ECI[1]
    x0[4] = Vel_ECI[2]
    x0[5] = Pos_ECI[0]
    x0[6] = Pos_ECI[1]
    x0[7] = Pos_ECI[2]
    x0[8] = 0.0
    x0[9] = 0.0
    x0[10] = 0.0
    x0[11] = Quat[0]
    x0[12] = Quat[1]
    x0[13] = Quat[2]
    x0[14] = Quat[3]

    print ('Initialize...Done')

    return x0
  
    
  t = np.arange(rocket.StartTime, rocket.EndTime + rocket.TimeStep, rocket.TimeStep)
  x0 = Initialize(rocket)
  params = np.array([WindSpeed, WindDirection])
  x = odeint(dynamics, x0, t, args=(params, rocket, result, ), mxstep=100000)

  return t, x

# TODO
# @ parachute
# @ Wind Log
# @ output


  
