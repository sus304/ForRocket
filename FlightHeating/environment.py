# -*- coding: utf-8 -*-
import numpy as np

# ref. 1976 standard atmosphere
# ジオポテンシャル高度を基準として標準大気の各層の気温減率から各大気値を算出
# 高度86 kmまで対応
def std_atmo(altitude):
  # altitude [m]
  R = 287.1
  gamma = 1.4
  Re = 6378.137e3 # Earth Radius [m]
  g0 = 9.80665

  # atmospheric layer
  height_list  = [0.0, 11.0e3, 20.0e3, 32.0e3, 47.0e3, 51.0e3, 71.0e3, 84.852e3] # geopotential height [m]
  temp_grad_list = [-6.5e-3, 0.0, 1.0e-3, 2.8e-3, 0, -2.8e-3, -2.0e-3, 0.0] # Temp. gradient [K/m]
  temp_list  = [288.15, 216.65, 216.65, 228.65, 270.65, 270.65, 214.65, 186.946] # [K]
  pressure_list  = [101325.0, 22632.0, 5474.9, 868.02, 110.91, 66.939, 3.9564, 0.3734] # [Pa]

  h = altitude * Re / (Re + altitude) # geometric altitude => geopotential height

  k = 0 # dafault layer
  for i in range(8):
    if h < height_list[i]:
      k = i - 1
      break
    elif h >= height_list[7]:
      k = 7
      break
  
  temperature = temp_list[k] + temp_grad_list[k] * (h - height_list[k]) # [K]
  if temp_grad_list[k] == 0.0:
    pressure = pressure_list[k] * np.exp(g0 / R * (height_list[k] - h) / temp_list[k])
  else:
    pressure = pressure_list[k] * np.power(temp_list[k] / temperature, g0 / R / temp_grad_list[k]) # [Pa]
  density = pressure / (R * temperature) # [kg/m^3]
  soundSpeed = np.sqrt(gamma * R * temperature) # [m/s]
  
  return temperature, pressure, density, soundSpeed

def std_temp(altitude):
  temp, press, rho, Cs = std_atmo(altitude)
  return temp
def std_pressure(altitude):
  temp, press, rho, Cs = std_atmo(altitude)
  return press
def std_density(altitude):
  temp, press, rho, Cs = std_atmo(altitude)
  return rho
def std_soundspeed(altitude):
  temp, press, rho, Cs = std_atmo(altitude)
  return Cs     

def gravity(altitude):
  # altitude [m]
  Re = 6378.137e3 # Earth Radius [m]
  g0 = 9.80665
  gravity = g0 * (Re / (Re + altitude)) ** 2 # [m/s^2]
  return gravity

if __name__ == '__main__':
  import matplotlib.pyplot as plt
  altitude = np.empty(0)
  dencity = np.empty(0)
  for alt in range(-1000, 87000, 100):
    T, rho = std_atmo(alt)
    g = gravity(100000.0)
    print (T, rho, g)
    altitude = np.append(altitude,alt)
    dencity = np.append(dencity, rho)

  plt.figure(0)
  plt.plot(altitude, dencity)
  plt.show()