# -*- coding: utf-8 -*-
import BarrowmanFlow as bf

################## User Input ###################
length_body = 3.9663 # [m] from nose tip to body. without tail
diameter_body = 0.183 # [m]
length_cg = 2.541 # [m] from nose tip

shape_nose = 'ogive' # 'ogive' or 'double' or 'parabolic' or 'ellipse'
length_nose = 0.640 # [m]

diameter_tail = 0.0 # [m]
length_tail = 0.0 # [m]

offset_fin = 0.0 # [mm] from body end to fin end
root_chord = 300.0 # [mm]
tip_chord = 150.0 # [mm]
leading_edge_chord = 150.0 #root_chord - tip_chord
span = 160.0 # [mm]
thickness_fin = 3.0 # [mm]
young_modulus = 69.0 # [GPa]
poisson_ratio = 0.3 # [-]
max_altitude = 6000.0 # [m]
#################################################

def mm2m(value):
  return value / 1000.0

offset_fin = mm2m(offset_fin)
root_chord = mm2m(root_chord)
tip_chord = mm2m(tip_chord)
leading_edge_chord = mm2m(leading_edge_chord)
span = mm2m(span)
thickness_fin = mm2m(thickness_fin)

bf.initialize(diameter_body, length_body)
nose = bf.Nose(shape_nose, length_nose)
fin = bf.Fin(root_chord, tip_chord, leading_edge_chord, span, length_body-offset_fin-root_chord)
fin.flutter_speed(young_modulus, poisson_ratio, thickness_fin, max_altitude)
# tail = bf.TaperBody(diameter_body, diameter_tail, length_tail, length_body)
stage = bf.integral(length_cg, nose, fin)

print('*=============Result==============*')
print('Length of C.P.:', stage.Lcp, '[m]')
print('Coefficient of Normal Force:', stage.CNa, '[deg^-1]')
print('Coefficient of Pitch Damping Moment:', stage.Cmq, '[-]')
print('Coefficient of Roll Damping Moment:', stage.Clp, '[-]')
print('Flutter Velocity:', max(fin.Vf), '[m/s]')
print('*=================================*')

stage.plot()



