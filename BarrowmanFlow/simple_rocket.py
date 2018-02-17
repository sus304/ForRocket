# -*- coding: utf-8 -*-
import BarrowmanFlow as bmf

################## User Input ###################
length_body = 1.743 # [m] from nose tip to body. without tail
diameter_body = 0.154 # [m]
mass_body = 13.238  # [kg]
length_cg = 1.07216 # [m] from nose tip

shape_nose = 'double' # 'ogive' or 'double' or 'parabolic' or 'ellipse'
length_nose = 0.2412 # [m]

diameter_tail = 0.0 # [m]
length_tail = 0.0 # [m]

offset_fin = 0.0 # [mm] from body end to fin end
root_chord = 190.0 # [mm]
tip_chord = 100.0 # [mm]
leading_edge_chord = 108.2 #root_chord - tip_chord
span = 130.0 # [mm]
thickness_fin = 2.0 # [mm]
young_modulus = 69.0 # [GPa]
poisson_ratio = 0.3 # [-]
rho_fin = 1380  # [kg/m3]
altitude = 700.0 # [m]
#################################################

def mm2m(value):
    return value / 1000.0

offset_fin = mm2m(offset_fin)
root_chord = mm2m(root_chord)
tip_chord = mm2m(tip_chord)
leading_edge_chord = mm2m(leading_edge_chord)
span = mm2m(span)
thickness_fin = mm2m(thickness_fin)

stage = bmf.Stage(diameter_body, length_body, length_cg)
nose = bmf.Nose(stage, shape_nose, length_nose)
fin = bmf.Fin(stage, root_chord, tip_chord, leading_edge_chord, span, length_body - offset_fin - root_chord)
fin.flutter_speed(young_modulus, poisson_ratio, thickness_fin, altitude)
length_cg = fin.center_of_gravity_for_fin(mass_body, length_cg, thickness_fin, rho_fin)
# tail = bmf.TaperBody(diameter_body, diameter_tail, length_tail, length_body)
stage.integrate([nose, fin])

print('*=============Result==============*')
print('Length of C.P.:', stage.Lcp, '[m]')
print('Length of C.G.:', length_cg, '[m]')
print('Safety Ratio of Length:', (stage.Lcp - length_cg) / length_body * 100.0, '[%]')
print('Coefficient of Normal Force:', stage.CNa, '[deg^-1]')
print('Coefficient of Pitch Damping Moment:', stage.Cmq, '[-]')
print('Coefficient of Roll Damping Moment:', stage.Clp, '[-]')
print('Flutter Velocity:', fin.Vf, '[m/s]')
print('Mass of Fin:', fin.mass_fin, '[kg]')
print('Mass of Body + Fin:', mass_body + fin.mass_fin, '[kg]')
print('*=================================*')

stage.plot()
