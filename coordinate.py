# -*- coding: utf-8 -*-
import numpy as np

def DCM_ECI2ECEF(t):
  # param t : time [sec]
  omega = 7.292115e-5 # Earth Angular Velocity on Equator [rad/s]
  theta = omega * t # [rad]

  DCM_ECI2ECEF = np.array([[np.cos(theta), np.sin(theta), 0.0], 
                          [-np.sin(theta), np.cos(theta), 0.0], 
                          [0.0           , 0.0          , 1.0]])
  return DCM_ECI2ECEF

def DCM_ECEF2NED(lat, lon):

  DCM_ECEF2NED = np.array([[-np.sin(lat) * np.cos(lon), -np.sin(lat) * np.sin(lon), np.cos(lat) ],
                           [-np.sin(lon)              , np.cos(lon)               , 0.0         ],
                           [-np.cos(lat) * np.cos(lon), -np.cos(lat) * np.sin(lon), -np.sin(lat)]])  
  return DCM_ECEF2NED

def Vector_ECI2ECEF_NEDframe(DCM_ECI2NED, Vector_ECI, Pos_ECI):
  # ECIの速度をECEF(NED frame)へ変換
  omega = 7.292115e-5 # Earth Angular Velocity on Equator [rad/s]
  tensor_ECI2ECEF = np.array([[0.0  , -omega, 0.0],
                              [omega, 0.0   , 0.0],
                              [0.0  , 0.0   , 0.0]])
  Vector_ECEF_NEDframe = DCM_ECI2NED.dot(Vector_ECI - tensor_ECI2ECEF.dot(Pos_ECI))
  return Vector_ECEF_NEDframe

def Vector_NED2ECI(DCM_NED2ECI, Vector_NED, Pos_ECI):
  # NEDの速度をECIへ変換
  omega = 7.292115e-5 # Earth Angular Velocity on Equator [rad/s]
  tensor_ECI2ECEF = np.array([[0.0  , -omega, 0.0],
                              [omega, 0.0   , 0.0],
                              [0.0  , 0.0   , 0.0]])
  return DCM_NED2ECI.dot(Vector_NED) + tensor_ECI2ECEF.dot(Pos_ECI)

def ECEF2LLH(Vector_ECEF):
  # ECEF座標から緯度経度高度に変換
  # Latitude-Longitude-Height
  # Vector_ECEF : [x, y, z]
  
  # WGS84 Constant
  a = 6378137.0
  f = 1.0 / 298.257223563
  b = a * (1.0 - f)
  e_sq = 2.0 * f - (f * f)
  e2_sq = (e_sq * a * a) / (b * b)
  
  p = np.sqrt(np.power(Vector_ECEF[0], 2) + np.power(Vector_ECEF[1], 2))
  theta = np.arctan2(Vector_ECEF[2] * a, p * b)
  Vector_LLH = np.zeros(3)
  Vector_LLH[0] = np.degrees(np.arctan2(Vector_ECEF[2] + e2_sq * b * np.power(np.sin(theta), 3),p - e_sq * a * np.power(np.cos(theta), 3)))
  Vector_LLH[1] = np.degrees(np.arctan2(Vector_ECEF[1], Vector_ECEF[0]))
  N = a / np.sqrt(1.0 - e_sq * np.power(np.sin(np.radians(Vector_LLH[0])), 2))
  Vector_LLH[2] = (p / np.cos(np.radians(Vector_LLH[0]))) - N

  return Vector_LLH

def LLH2ECEF(Vector_LLH):
  # Vector_LLH : [latitude, longitude, height] = [deg, deg, m]
  
  # WGS84 Constant
  a = 6378137.0
  f = 1.0 / 298.257223563
  e_sq = 2.0 * f - (f * f)
  N = a / np.sqrt(1.0 - e_sq * np.power(np.sin(np.radians(Vector_LLH[0])), 2))
  Vector_ECEF = np.zeros(3)  
  Vector_ECEF[0] = (N + Vector_LLH[2]) * np.cos(np.radians(Vector_LLH[0])) * np.cos(np.radians(Vector_LLH[1]))
  Vector_ECEF[1] = (N + Vector_LLH[2]) * np.cos(np.radians(Vector_LLH[0])) * np.sin(np.radians(Vector_LLH[1]))
  Vector_ECEF[2] = (N * (1.0 - e_sq) + Vector_LLH[2]) * np.sin(np.radians(Vector_LLH[0]))
  
  return Vector_ECEF

def DCM_Body2Air(alpha, beta):  
  DCM_Body2Air = np.array([[np.cos(alpha) * np.cos(beta) , np.sin(beta), np.sin(alpha) * np.cos(beta) ],
                           [-np.cos(alpha) * np.sin(beta), np.cos(beta), -np.sin(alpha) * np.sin(beta)],
                           [-np.sin(alpha)               , 0.0         , np.cos(alpha)                ]])
  return DCM_Body2Air

def DCM_NED2Body_Euler(Azimuth, Elevation, Roll):
  DCM_NED2Body_Euler = np.array([[np.cos(Azimuth) * np.cos(Elevation), np.sin(Azimuth) * np.cos(Elevation), -np.sin(Elevation)],
                                 [np.cos(Azimuth) * np.sin(Elevation) * np.sin(Roll) - np.sin(Azimuth) * np.cos(Roll), np.sin(Azimuth) * np.sin(Elevation) * np.sin(Roll) + np.cos(Azimuth) * np.cos(Roll), np.cos(Elevation) * np.sin(Roll)],
                                 [np.cos(Azimuth) * np.sin(Elevation) * np.cos(Roll) + np.sin(Azimuth) * np.sin(Roll), np.sin(Azimuth) * np.sin(Elevation) * np.cos(Roll) - np.cos(Azimuth) * np.sin(Roll), np.cos(Elevation) * np.cos(Roll)]])
  return DCM_NED2Body_Euler

def Quat_normalize(Quat):
  norm = np.linalg.norm(Quat)
  Quat = Quat / norm
  return Quat

def DCM_NED2Body_Quat(Quat):
  q0 = Quat[0]
  q1 = Quat[1]
  q2 = Quat[2]
  q3 = Quat[3]

  DCM_NED2Body_Quat = np.array([[q0*q0 + q1*q1 - q2*q2 - q3*q3, 2.0 * (q1 * q2 + q0 * q3)    , 2.0 * (q1 * q3 - q0 * q2)],
                                [2.0 * (q1 * q2 - q0 * q3)    , q0*q0 - q1*q1 + q2*q2 - q3*q3, 2.0 * (q2 * q3 + q0 * q1)],
                                [2.0 * (q1 * q3 + q0 * q2)    , 2.0 * (q2 * q3 - q0 * q1)    , q0*q0 - q1*q1 - q2*q2 + q3*q3]])
  return DCM_NED2Body_Quat

def Euler2Quat(Azimuth, Elevation, Roll=0.0):
  Azimuth2 = np.radians(Azimuth * 0.5)
  Elevation2 = np.radians(Elevation * 0.5)
  Roll2 = np.radians(Roll * 0.5)

  q0 = np.cos(Azimuth2) * np.cos(Elevation2) * np.cos(Roll2) + np.sin(Azimuth2) * np.sin(Elevation2) * np.sin(Roll2)
  q1 = np.cos(Azimuth2) * np.cos(Elevation2) * np.sin(Roll2) - np.sin(Azimuth2) * np.sin(Elevation2) * np.cos(Roll2)
  q2 = np.cos(Azimuth2) * np.sin(Elevation2) * np.cos(Roll2) + np.sin(Azimuth2) * np.cos(Elevation2) * np.sin(Roll2)
  q3 = np.sin(Azimuth2) * np.cos(Elevation2) * np.cos(Roll2) - np.cos(Azimuth2) * np.sin(Elevation2) * np.sin(Roll2)

  Quat = np.array([q0, q1, q2, q3])
  Quat = Quat_normalize(Quat)

  return Quat

def Quat2Euler(DCM_NED2Body_Quat):
  Azimuth = np.arctan2(DCM_NED2Body_Quat[0,1], DCM_NED2Body_Quat[0,0])
  Elevation = np.arcsin(-DCM_NED2Body_Quat[0,2])
  Roll = np.arctan2(DCM_NED2Body_Quat[2,1], DCM_NED2Body_Quat[2,2])

  return Azimuth, Elevation, Roll