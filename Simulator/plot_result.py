# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt
# import simplekml

class ResultBox:
  def __init__(self):
    self.items = []

    # Output Items
    self.items.append('Time [s]') # 0
    self.items.append('Mf [kg]') # 1
    self.items.append('Mox [kg]') # 2
    self.items.append('M [kg]') # 3
    self.items.append('Mdot_ox [kg/s]') # 4
    self.items.append('Mdot_f [kg/s]') # 5
    self.items.append('Lcgp [m]') # 6
    self.items.append('Lcg [m]') # 7
    self.items.append('Isp [s]') # 8    
    self.items.append('Thrust [N]') # 9
    self.items.append('Drag [N]') # 10
    self.items.append('Normal Y [N]') # 11
    self.items.append('Normal Z [N]') # 12
    self.items.append('Acc_ECI X [m/s^2]') # 13
    self.items.append('Acc_ECI Y [m/s^2]') # 14
    self.items.append('Acc_ECI Z [m/s^2]') # 15
    self.items.append('Vel_ECI X [m/s]') # 16
    self.items.append('Vel_ECI Y [m/s]') # 17
    self.items.append('Vel_ECI Z [m/s]') # 18
    self.items.append('Pos_ECI X [m]') # 19
    self.items.append('Pos_ECI Y [m]') # 20
    self.items.append('Pos_ECI Z [m]') # 21
    self.items.append('Pos_ECEF X [m]') # 22
    self.items.append('Pos_ECEF Y [m]') # 23
    self.items.append('Pos_ECEF Z [m]') # 24
    self.items.append('Latitude [deg]') # 25
    self.items.append('Longitude [deg]') # 26
    self.items.append('Height [m]') # 27
    self.items.append('Azimuth [deg]') # 28
    self.items.append('Elevation [deg]') # 29
    self.items.append('Wind_North [m/s]') # 30
    self.items.append('Wind_East [m/s]') # 31
    self.items.append('Vel_Air X [m/s]') # 32
    self.items.append('Vel_Air Y [m/s]') # 33
    self.items.append('Vel_Air Z [m/s]') # 34
    self.items.append('alpha [deg]') # 35
    self.items.append('beta [deg]') # 36
    self.items.append('Mach [-]') # 37
    self.items.append('DynamicPressure [Pa]') # 38
    self.items.append('g [m/s^2]') # 39
    self.items.append('Pa [Pa]') # 40
    self.items.append('rho [kg/m^3]') # 41
    self.items.append('Moment X [Nm]') # 42
    self.items.append('Moment Y [Nm]') # 43
    self.items.append('Moment Z [Nm]') # 44
    self.items.append('Omega X [rad/s]') # 45
    self.items.append('Omega Y [rad/s]') # 46
    self.items.append('Omega Z [rad/s]') # 47

    self.value = np.zeros(len(self.items))

  def tdebug(self, de):
    self.value = np.delete(self.value, 0, 0)

    plt.close('all')
    plt.figure(0)
    for i in range(1,4):
      plt.plot(self.value[:,0], self.value[:,i], label=self.items[i])
    plt.legend(loc='best')
    plt.grid()
    plt.figure(1)
    for i in range(4,6):
      plt.plot(self.value[:,0], self.value[:,i], label=self.items[i])
    plt.legend(loc='best')
    plt.grid()
    plt.figure(2)
    for i in range(6,8):
      plt.plot(self.value[:,0], self.value[:,i], label=self.items[i])
    plt.legend(loc='best')
    plt.grid()
    plt.figure(3)
    for i in range(8,9):
      plt.plot(self.value[:,0], self.value[:,i], label=self.items[i])
    plt.legend(loc='best')
    plt.grid()
    plt.figure(4)
    for i in range(9,13):
      plt.plot(self.value[:,0], self.value[:,i], label=self.items[i])
    plt.legend(loc='best')
    plt.grid()
    plt.figure(5)
    for i in range(13,16):
      plt.plot(self.value[:,0], self.value[:,i], label=self.items[i])
    plt.legend(loc='best')
    plt.grid()
    plt.figure(6)
    for i in range(16,19):
      plt.plot(self.value[:,0], self.value[:,i], label=self.items[i])
    plt.legend(loc='best')
    plt.grid()
    plt.figure(7)
    for i in range(19,22):
      plt.plot(self.value[:,0], self.value[:,i], label=self.items[i])
    plt.legend(loc='best')
    plt.grid()
    plt.figure(8)
    for i in range(22,25):
      plt.plot(self.value[:,0], self.value[:,i], label=self.items[i])
    plt.legend(loc='best')
    plt.grid()
    plt.figure(9)
    for i in range(27,28):
      plt.plot(self.value[:,0], self.value[:,i], label=self.items[i])
    plt.legend(loc='best')
    plt.grid()
    plt.figure(10)
    for i in range(28,30):
      plt.plot(self.value[:,0], self.value[:,i], label=self.items[i])
    plt.legend(loc='best')
    plt.grid()
    plt.figure(11)
    for i in range(30,32):
      plt.plot(self.value[:,0], self.value[:,i], label=self.items[i])
    plt.legend(loc='best')
    plt.grid()
    plt.figure(12)
    for i in range(32,35):
      plt.plot(self.value[:,0], self.value[:,i], label=self.items[i])
    plt.legend(loc='best')
    plt.grid()
    plt.figure(13)
    for i in range(35,37):
      plt.plot(self.value[:,0], self.value[:,i], label=self.items[i])
    plt.legend(loc='best')
    plt.grid()
    plt.figure(14)
    for i in range(37,38):
      plt.plot(self.value[:,0], self.value[:,i], label=self.items[i])
    plt.legend(loc='best')
    plt.grid()
    plt.figure(15)
    for i in range(38,39):
      plt.plot(self.value[:,0], self.value[:,i], label=self.items[i])
    plt.legend(loc='best')
    plt.grid()
    plt.figure(16)
    for i in range(45,48):
      plt.plot(self.value[:,0], self.value[:,i], label=self.items[i])
    plt.legend(loc='best')
    plt.grid()

    plt.show()

# def kml_make(name,Launch_LLH):
#   Log = np.loadtxt('Position_log.csv',delimiter=",",skiprows=1)
#   array = Log[:,1]
#   array_len = len(array)
#   print ":"
#   Position_ENU = np.zeros((array_len,3))
#   Position_ENU[:,0] = np.array(Log[:,0])
#   Position_ENU[:,1] = np.array(Log[:,1])
#   Position_ENU[:,2] = np.array(Log[:,2])
  
#   Position_ecef = np.zeros((array_len,3))
#   Position_LLH = np.zeros((array_len,3))
#   print ":"
#   for i in range(array_len):
#     Position_ecef[i,:] = ENU2ECEF(Position_ENU[i,:],Launch_LLH)
#     Position_LLH[i,:] = ECEF2LLH(Position_ecef[i,:])
#   print ":"
  
#   header = 'Latitude,Longitude,Height'
#   np.savetxt("Result Log 1.csv",Position_LLH,fmt = '%.5f',delimiter = ',',header = header)
  
#   kml = simplekml.Kml(open=1)
#   Log_LLH = []
#   for i in range(array_len):
#     if 0 == i % 10000:
#       Log_LLH.append((Position_LLH[i,1],Position_LLH[i,0],Position_LLH[i,2]))
#   print ":"
#   line = kml.newlinestring(name = name)
#   line.style.linestyle.width = 5
#   line.style.linestyle.color = simplekml.Color.red
#   line.extrude = 1
#   line.altitudemode = simplekml.AltitudeMode.absolute
#   line.coords = Log_LLH
#   line.style.linestyle.colormode = simplekml.ColorMode.random
#   kml.save(name + ".kml")



# try:
#   kml_make(name,Launch_LLH)
#   os.chdir("../")
# except:
#   print "Error : Can't make kml file...script exit"
#   os.chdir("../")

# print "Script Complete!"
