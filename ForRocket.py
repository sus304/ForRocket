# -*- coding: utf-8 -*-
import os
import sys
import json

print ('6-DoF Rocket Flight Simulator')
print ('ForRocket')

import rocket_dynamics as Rocket
import plot_result as Result

# Config File Load
argv = sys.argv
if len(argv) < 2:
  print ('Argument is missing')
  print ('Usage : python ForRocket.py configFileName.json')
  sys.exit()
print('Config File : %s > ...Loading' % argv[1])
ConfigFile = open(argv[1])
json_obj = json.load(ConfigFile)

# Result Directory Create
ObjectName = json_obj.get('Name')
ResultDir = json_obj.get('System').get('Result Directory') + '_' + ObjectName
if os.path.exists(ResultDir):  
  ResultDir_org = ResultDir
  i = 1
  while os.path.exists(ResultDir):
    sys.stderr.write ('\r\033[K' + '%s already exists' % ResultDir)
    sys.stderr.flush()
    ResultDir = ResultDir_org + '_%02d' % (i)
    i = i + 1
else:
  print ('"%s" not found' % ResultDir)
print ()
print ('Making...')
os.mkdir(ResultDir)
print ('Result Directory : %s' % ResultDir)


# Rocket instance
rocket = Rocket.Config(json_obj)
result = Result.ResultBox()
TimeLog, Log = Rocket.Simulation(rocket, 2.0, 45.0, ResultDir, result)
result.debug()

