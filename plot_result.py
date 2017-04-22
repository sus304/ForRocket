# -*- coding: utf-8 -*-
import numpy as np
# import simplekml

class ResultBox:
  def __init__(self):
    self.items = []

    self.items.append('Time [s]')
    self.items.append('Mf [kg]')
    self.items.append('Mox [kg]')
    self.items.append('M [kg]')
    self.items.append('Lcgox [m]')
    self.items.append('Lcg [m]')
    self.items.append('Lcp [m]')

 


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
