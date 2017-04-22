# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt
# import simplekml

class ResultBox:
  def __init__(self):
    self.items = []

    # Output Items
    self.items.append('Time [s]')
    self.items.append('Mf [kg]')
    self.items.append('Mox [kg]')
    self.items.append('M [kg]')
    self.items.append('Mdot_ox [kg/s]')
    self.items.append('Mdot_f [kg/s]')
    self.items.append('Lcgp [m]')
    self.items.append('Lcg [m]')
    self.items.append('Thrust [N]')
    self.items.append('Isp [s]')
    self.items.append('Drag [N]')
    self.items.append('Normal Y [N]')
    self.items.append('Normal Z [N]')
    self.items.append('Acc_ECI X [m/s^2]')
    self.items.append('Acc_ECI Y [m/s^2]')
    self.items.append('Acc_ECI Z [m/s^2]')
    self.items.append('Vel_ECI X [m/s]')
    self.items.append('Vel_ECI Y [m/s]')
    self.items.append('Vel_ECI Z [m/s]')
    self.items.append('Pos_ECI X [m]')
    self.items.append('Pos_ECI Y [m]')
    self.items.append('Pos_ECI Z [m]')
    self.items.append('Pos_ECEF X [m]')
    self.items.append('Pos_ECEF Y [m]')
    self.items.append('Pos_ECEF Z [m]')
    self.items.append('Latitude [deg]')
    self.items.append('Longitude [deg]')
    self.items.append('Height [m]')
    self.items.append('Azimuth [deg]')
    self.items.append('Elevation [deg]')
    self.items.append('Wind_North [m/s]')
    self.items.append('Wind_East [m/s]')
    self.items.append('Vel_Air X [m/s]')
    self.items.append('Vel_Air Y [m/s]')
    self.items.append('Vel_Air Z [m/s]')
    self.items.append('alpha [deg]')
    self.items.append('beta [deg]')
    self.items.append('Mach [-]')
    self.items.append('DynamicPressure [Pa]')
    self.items.append('g [m/s^2]')
    self.items.append('Pa [Pa]')
    self.items.append('rho [kg/m^3]')
    self.items.append('Ij X [kg*m^2]')
    self.items.append('Ij Y [kg*m^2]')
    self.items.append('Ij Z [kg*m^2]')
    self.items.append('Moment X [Nm]')
    self.items.append('Moment Y [Nm]')
    self.items.append('Moment Z [Nm]')
    self.items.append('Omega X [rad/s]')
    self.items.append('Omega Y [rad/s]')
    self.items.append('Omega Z [rad/s]')

    self.value = np.empty(len(self.items))

  def debug():
    # self.value = np.delete(self.value, 0, 0)

    plt.close('all')
    plt.figure(0)
    plt.plot(self.value[:,0], self.value[:,1], label='Mf')
    plt.plot(self.value[:,0], self.value[:,2], label='Mox')
    plt.plot(self.value[:,0], self.value[:,3], label='M')
    plt.plot(self.value[:,0], self.value[:,4], label='Lcgp')
    plt.plot(self.value[:,0], self.value[:,5], label='Lcg')
    plt.plot(self.value[:,0], self.value[:,6], label='Lcp')
    plt.legend(loc='best')
    plt.grid()
    
    plt.figure(1)
    plt.plot(self.value[:,0], self.value[:,7], label='Drag')
    plt.plot(self.value[:,0], self.value[:,8], label='Thrust')
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
