# -*- coding: utf-8 -*-
import numpy as np
import BarrowmanFlow as bmf

class Config:
    def __init__(self):
        ################## User Input ###################
        self.length_body = 2.3 # [m] from nose tip to body. without tail
        self.diameter_body = 0.154 # [m]
        self.mass_body = 12.9  # [kg]
        self.length_cg = 1.294 # [m] from nose tip

        self.shape_nose = 'double' # 'ogive' or 'double' or 'parabolic' or 'ellipse'
        self.length_nose = 0.2412 # [m]

        self.diameter_tail = 0.0 # [m]
        self.length_tail = 0.0 # [m]

        self.offset_fin = 0.0 / 1000.0 # [m] from body end to fin end
        self.root_chord = 150.0 / 1000.0 # [m]

        self.tip_chord_max = 500.0 / 1000.0 # [m]
        self.tip_chord_min = 50.0 / 1000.0 # [m]
        self.leading_edge_chord_max = 500.0 / 1000.0 # [m]
        self.leading_edge_chord_min = 10.0 / 1000.0 # [m]
        # self.span_max = self.diameter_body # [m]
        self.span_max = 130.0e-3 # [m]
        self.span_min = 10.0 / 1000.0 # [m]
        
        self.thickness_fin = 2.0 / 1000.0 # [m]
        self.young_modulus = 69.0 # [GPa]
        self.poisson_ratio = 0.3 # [-]
        self.rho_fin = 1380        
        self.altitude = 842.0 # [m]

        self.Fst_max = 20.0
        self.Fst_min = 12.0
        self.theta_max = 8.0 # [deg] 後端後退角
        self.theta_min = -50.0 # [deg] 後端後退角

        self.stage = bmf.Stage(self.diameter_body, self.length_body, self.length_cg)
        self.nose = bmf.Nose(self.stage, self.shape_nose, self.length_nose)
        self.components = [self.nose]
        #################################################


def equality(param, config):
    root = param[0]
    tip = param[1]
    leading = param[2]
    span = param[3]
    offset = param[4]

    condition = bmf.Conditions()
    
    condition.equal(offset, config.offset_fin)
    condition.equal(root, config.root_chord)

    return condition()

def inequality(param, config):
    root = param[0]
    tip = param[1]
    leading = param[2]
    span = param[3]
    offset = param[4]

    fin = bmf.Fin(config.stage, root, tip, leading, span, config.length_body - offset - root)
    components = []
    [components.append(compo) for compo in config.components]
    components.append(fin)
    config.stage.integrate(components)
    length_cp = config.stage.Lcp
    length_cg = fin.center_of_gravity_for_fin(config.mass_body, config.length_cg, config.thickness_fin, config.rho_fin)
    Fst = (length_cp - length_cg) / config.length_body * 100.0
    # Fst = (length_cp - config.length_cg) / config.length_body * 100.0
    theta = np.arctan2(tip + leading - root, span)
    theta = np.rad2deg(theta)
    
    condition = bmf.Conditions()

    condition.upper_bound(tip, config.tip_chord_max)
    condition.upper_bound(leading, config.leading_edge_chord_max)
    condition.upper_bound(span, config.span_max)
    condition.upper_bound(Fst, config.Fst_max)
    condition.upper_bound(theta, config.theta_max)

    condition.lower_bound(tip, config.tip_chord_min)
    condition.lower_bound(leading, config.leading_edge_chord_min)
    condition.lower_bound(span, config.span_min)
    condition.lower_bound(Fst, config.Fst_min)
    condition.lower_bound(theta, config.theta_min)

    return condition()       

def cost(param, config):
    root = param[0]
    tip = param[1]
    leading = param[2]
    span = param[3]
    offset = param[4]

    A_fin = (root + tip) * span * 0.5
    return A_fin
        
config = Config()        
opt = bmf.FinOptimize()
root_init = 0.1
tip_init = 0.1
leading_init = 0.1
span_init = 0.1
offset_init = 0.0
opt.set_param_init([root_init, tip_init, leading_init, span_init, offset_init])
opt.equality = equality
opt.inequality = inequality
opt.cost = cost
result = opt.solve(config)

print(result.message)
root_chord = result.x[0]
tip_chord = result.x[1]
leading_edge_chord = result.x[2]
span = result.x[3]
offset_fin = result.x[4]

fin = bmf.Fin(config.stage, root_chord, tip_chord, leading_edge_chord, span, config.length_body - offset_fin - root_chord)
length_cg = fin.center_of_gravity_for_fin(config.mass_body, config.length_cg, config.thickness_fin, config.rho_fin)
# fin.flutter_speed(young_modulus, poisson_ratio, thickness_fin, altitude)
# tail = bmf.TaperBody(diameter_body, diameter_tail, length_tail, length_body)
config.stage.integrate([config.nose, fin])

print('*=============Result==============*')
print('Length of C.P.:', config.stage.Lcp, '[m]')
print('Length of C.G.:', config.length_cg, '[m]')
print('Safety Ratio of Length:', (config.stage.Lcp - length_cg) / config.length_body * 100.0, '[%]')
print('Coefficient of Normal Force:', config.stage.CNa, '[deg^-1]')
print('Coefficient of Pitch Damping Moment:', config.stage.Cmq, '[-]')
print('Coefficient of Roll Damping Moment:', config.stage.Clp, '[-]')
# print('Flutter Velocity:', fin.Vf, '[m/s]')
print('Mass of Fin:', fin.mass_fin, '[kg]')
print('Mass of Body + Fin:', config.mass_body + fin.mass_fin, '[kg]')
print('*=================================*')

print('*=============Size==============*')
print('Root Chord:', root_chord, '[m]')
print('Tip Chord:', tip_chord, '[m]')
print('Leading Edge Chord:', leading_edge_chord, '[m]')
print('Span:', span, '[m]')
print('Offset:', offset_fin, '[m]')
print('*=================================*')

config.stage.plot()