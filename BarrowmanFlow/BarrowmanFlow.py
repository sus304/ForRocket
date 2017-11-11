# -*- coding: utf-8 -*-
'''
MIT License
Copyright (c) 2017 Susumu Tanaka

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
'''
import re
import numpy as np
import matplotlib.pyplot as plt

class AeroObj:
  d_body = 0.0
  l_body = 0.0

  def __init__(self):
    if AeroObj.d_body <= 0.0 or AeroObj.l_body <= 0.0:
      print('Error:Not initialized. BarrowmanFlow.initialize(...)')
    self.CNa = 0.0
    self.inertia_coefficient = 0.0
    self.Cmq = 0.0
    self.Clp = 0.0
    self.Cnr = 0.0
    self.Lcp = 0.0

class Nose(AeroObj):
  def __init__(self, shape, l_nose):
    super().__init__()
    self.LD = l_nose / AeroObj.d_body # copy for graph plot

    doublecone_pattern = r'double|double ?cone|cone'
    ogive_pattern = r'ogive|ogive ?cone'
    parabolic_pattern = r'parabolic|ellipse'
    if re.compile(doublecone_pattern, re.IGNORECASE).match(shape):
      CP_coefficient = 2.0 / 3.0
    elif re.compile(ogive_pattern, re.IGNORECASE).match(shape):
      CP_coefficient = 1.0 - ((8.0 * self.LD ** 2 / 3.0) + ((4.0 * self.LD ** 2 - 1.0) ** 2 / 4.0) - (((4.0 * self.LD ** 2 - 1.0) * (4.0 * self.LD ** 2 + 1.0) ** 2 / (16.0 * self.LD)) * np.arcsin(4.0 * self.LD / (4.0 * self.LD ** 2 + 1.0))))
    elif re.compile(parabolic_pattern, re.IGNORECASE).match(shape):
      CP_coefficient =  0.5
    
    self.CNa = 2.0
    self.Lcp = l_nose * CP_coefficient

class TaperBody(AeroObj):
  def __init__(self, d_before, d_after, l_taper, distance_fromNoseTip):
    super().__init__()
    self.d_before = d_before # copy for graph plot
    self.d_after = d_after
    self.l_taper = l_taper
    self.distance = distance_fromNoseTip

    self.CNa = 2.0 * ((d_after / d_before) ** 2 - 1.0)
    self.Lcp = distance_fromNoseTip + (l_taper / 3.0) * (1.0 + ((1.0 - (d_before / d_after)) / (1.0 - (d_before / d_after) ** 2)))


class Fin(AeroObj):
  def __init__(self, Cr, Ct, Cle, span, distance_fromNoseTip_toRootchordLeadingEdge):
    super().__init__()
    # input unit [m]
    # Cr:Root Chord
    # Ct:Tip Chord
    # Cle:Leading Edge Chord
    self.Cr = Cr # copy for graph plot
    self.Ct = Ct
    self.Cle = Cle
    self.span = span
    self.distance = distance_fromNoseTip_toRootchordLeadingEdge
    
    # フィン形状による分岐
    if Cle+0.5*Ct == 0.5*Cr:
      mid_chord_line = span
    elif Cle+0.5*Ct > 0.5*Cr:
      mid_chord_line = np.sqrt(span ** 2 + (0.5 * Ct + Cle - 0.5 * Cr) ** 2)
    else:
      mid_chord_line = np.sqrt(span ** 2 + (0.5 * Cr - Cle - 0.5 * Ct) ** 2)

    CNa_single = 16.0 * (span / AeroObj.d_body) ** 2 / (1.0 + np.sqrt(1.0 + (2.0 * mid_chord_line / (Cr + Ct)) ** 2)) # 4fins
    Kfb = 1.0 + 0.5 * AeroObj.d_body / (0.5 * AeroObj.d_body + span) # interference fin and body
    self.CNa = CNa_single * Kfb

    ramda = Ct / Cr
    MAC = 2.0 / 3.0 * Cr * (1 + ramda ** 2 / (1.0 + ramda)) # Mean Aerodynamic Chord
    self.Lcp = self.distance + (Cle * (Cr + 2.0 * Ct) / (3.0 * (Cr + Ct))) + MAC / 4.0
    self.Clp = -4.0 * 2.0 * (span + 0.5 * AeroObj.d_body) ** 4 / (np.pi * AeroObj.l_body ** 2 * (0.25 * AeroObj.d_body ** 2 * np.pi))

  def flutter_speed(self, young, poisson, thickness, altitude=0.0):
    # ref. NACA Technical Note 4197
    # young:Young`s modulus [GPa]
    # possion:Poisson Ratio
    # thickness:Fin thickness [m]

    def Std_Atmo(altitude):
      # ref. 1976 standard atmosphere
      # ジオポテンシャル高度を基準として標準大気の各層の気温減率から各大気値を算出
      # 高度86 kmまで対応
      # altitude [m]
      R = 287.1
      gamma = 1.4
      Re = 6378.137e3 # Earth Radius [m]
      g0 = 9.80665

      # atmospheric layer
      h_list  = [0.0, 11.0e3, 20.0e3, 32.0e3, 47.0e3, 51.0e3, 71.0e3, 84.852e3] # geopotential height [m]
      TG_list = [-6.5e-3, 0.0, 1.0e-3, 2.8e-3, 0, -2.8e-3, -2.0e-3, 0.0] # Temp. gradient [K/m]
      T_list  = [288.15, 216.65, 216.65, 228.65, 270.65, 270.65, 214.65, 186.946] # [K]
      P_list  = [101325.0, 22632.0, 5474.9, 868.02, 110.91, 66.939, 3.9564, 0.3734] # [Pa]

      h = altitude * Re / (Re + altitude) # geometric altitude => geopotential height

      k = 0 # dafault layer
      for i in range(8):
        if h < h_list[i]:
          k = i - 1
          break
        elif h >= h_list[7]:
          k = 7
          break
      
      temperature = T_list[k] + TG_list[k] * (h - h_list[k]) # [K]
      if TG_list[k] == 0.0:
        pressure = P_list[k] * np.exp(g0 / R * (h_list[k] - h) / T_list[k])
      else:
        pressure = P_list[k] * np.power(T_list[k] / temperature, g0 / R / TG_list[k]) # [Pa]
      density = pressure / (R * temperature) # [kg/m^3]
      soundspeed = np.sqrt(gamma * R * temperature) # [m/s]
      
      return temperature, pressure, density, soundspeed

    AR = 2.0 * self.span / (self.Cr + self.Ct) # aspect ratio
    ramda = self.Ct / self.Cr # taper ratio
    shear = young / (2.0 * (1.0 + poisson)) # shear modulus
    # Alt 0.0m から指定高度まで
    self.Vf = [Std_Atmo(alt)[3] * np.sqrt(shear * 10.0 ** 9 / ((1.337 * AR ** 3 * Std_Atmo(alt)[1] * (ramda + 1.0)) / (2.0 * (AR + 2.0) * (thickness / self.Cr) ** 3))) for alt in np.arange(0.0, altitude+10.0, 10.0)]

class Stage:
  def __init__(self, d_body, l_body, Lcg, Lcp, CNa, Cmq, Cnr, Clp):
    self.d_body = d_body
    self.l_body = l_body
    self.Lcg = Lcg
    self.Lcp = Lcp
    self.CNa = CNa
    self.Cmq = Cmq
    self.Cnr = Cnr
    self.Clp = Clp
    self.components = []
  
  def plot(self):
    graph = Graph(self)
    graph.plot()

def initialize(d_body, l_body):
  AeroObj.d_body = d_body
  AeroObj.l_body = l_body

def integral(Lcg, *components):
  stage = Stage(AeroObj.d_body, AeroObj.l_body, Lcg, 0.0, 0.0, 0.0, 0.0, 0.0)
  for obj in components:
    stage.CNa += obj.CNa
    stage.Lcp += obj.CNa * obj.Lcp
    stage.Cmq -= 4.0 * (0.5 * obj.CNa * ((obj.Lcp - Lcg) / AeroObj.l_body) ** 2)
    stage.Clp += obj.Clp
    stage.components.append(obj)
  stage.Lcp /= stage.CNa

  return stage

class Graph:
  # グラフによる機体形状の可視化
  # 単純な単段ロケットのみ対応
  # @ToDo:汎用性を上げる
  def add_point(self, array, x, y):
    return np.vstack((array, np.array([x, y])))

  def add_body_reverse(self, point_list):
    point_parse = point_list[:-1,:]
    for point in point_parse[::-1]:
      point_list = self.add_point(point_list, point[0], point[1] * (-1))
    return point_list

  def add_fin_reverse(self, point_list):
    point_list = self.add_point(point_list, point_list[0,0], point_list[0,1]) # 1st fin close   
    point_2nd = np.array([point_list[0,0], -point_list[0,1]])
    for point in point_list[1:]:
      point_2nd = self.add_point(point_2nd, point[0], point[1] * (-1)) # 2nd fin point
    return point_list, point_2nd

  def __init__(self, stage):
    self.Lcg = stage.Lcg
    self.Lcp = stage.Lcp
    self.point_set = []
    for component in stage.components:
      if isinstance(component, Nose):
        if hasattr(self, 'point_nose'):
          pass
        else:
          self.point_nose = np.array([0.0, 0.0])
        self.point_nose = self.add_point(self.point_nose, component.LD*stage.d_body, 0.5*stage.d_body)
        self.point_nose = self.add_point(self.point_nose, component.LD*stage.d_body, 0.0)
        self.point_nose = self.add_body_reverse(self.point_nose)
        self.point_set.append(self.point_nose)       

      elif isinstance(component, TaperBody):
        if hasattr(self, 'point_taper'):
          pass
        else:
          self.point_taper = np.array([component.distance, 0.0])
        self.point_taper = self.add_point(self.point_taper, component.distance, 0.5*component.d_before)
        self.point_taper = self.add_point(self.point_taper, component.distance+component.l_taper, 0.5*component.d_after)
        self.point_taper = self.add_point(self.point_taper, component.distance+component.l_taper, 0.0)
        self.point_taper = self.add_body_reverse(self.point_taper)
        self.point_set.append(self.point_taper)               

      elif isinstance(component, Fin):
        if hasattr(self, 'point_fin'):
          pass
        else:
          self.point_fin = np.array([component.distance, 0.5*stage.d_body])
        self.point_fin = self.add_point(self.point_fin, component.distance+component.Cle, 0.5*stage.d_body+component.span)
        self.point_fin = self.add_point(self.point_fin, component.distance+component.Cle+component.Ct, 0.5*stage.d_body+component.span)
        self.point_fin = self.add_point(self.point_fin, component.distance+component.Cr, 0.5*stage.d_body)
        self.point_fin, self.point_fin_2nd = self.add_fin_reverse(self.point_fin)
        self.point_set.append(self.point_fin)
        self.point_set.append(self.point_fin_2nd)                      
        
    # add body
    start_x = max(self.point_nose[:,0])
    end_x = AeroObj.l_body
    self.point_body = np.array([start_x, -0.5*stage.d_body])
    self.point_body = self.add_point(self.point_body, start_x, 0.5*stage.d_body)
    self.point_body = self.add_point(self.point_body, end_x, 0.5*stage.d_body)
    self.point_body = self.add_point(self.point_body, end_x, -0.5*stage.d_body)
    self.point_body = self.add_point(self.point_body, start_x, -0.5*stage.d_body)
    self.point_set.append(self.point_body)           

  def plot(self):
    plt.close('all')
    plt.figure(0, figsize=(8, 4))
    xmax = 0.0
    ymax = 0.0
    for point_list in self.point_set:
      try:
        plt.plot(point_list[:,0], point_list[:,1], color='black')
        if max(point_list[:,0]) > xmax:
          xmax = max(point_list[:,0])
        if max(point_list[:,1]) > ymax:
          ymax = max(point_list[:,1])
      except:
        pass
    plt.plot(self.Lcg, 0.0, 'o', color='black', label='Lcg')
    plt.plot(self.Lcp, 0.0, 'o', color='red', label='Lcp')
    plt.xlim([0.0, np.ceil(xmax)])
    plt.ylim([np.floor(-ymax), np.ceil(ymax)])
    ax = plt.gca()
    aspect = 1.0
    ax.set_aspect(aspect)
    plt.grid()
    plt.legend()
    plt.show()
