# -*- coding: utf-8 -*-

import platform
import subprocess as sp

print "Simulation Start!"

print "cs2data"
import cs2data
print ""
print "Calcutate"
if 'Windows' == platform.system():
	sp.call(['Simulation'])
else:
	sp.call(['./Simulation.exe'])
print ""
print "Simulation End!"