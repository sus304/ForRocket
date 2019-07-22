import datetime
import numpy as np
import pymap3d as pm

import Simulator.coordinate as coord
import Simulator.environment as env

def parachute_dynamics(x, t, rocket):
    pos_ECI = x[0:3]
    vel_ECI = x[3:6]

    date_current = rocket.launch_date + datetime.timedelta(seconds=t)
    
    pos_LLH = pm.eci2geodetic(pos_ECI, date_current)
    altitude = pos_LLH[2]
    DCM_ECI2ECEF = coord.DCM_ECI2ECEF(t)
    DCM_ECEF2NED = coord.DCM_ECEF2NED(rocket.pos0_LLH)
    vel_ECEF = coord.vel_ECI2ECEF(vel_ECI, DCM_ECI2ECEF, pos_ECI)
    vel_NED = DCM_ECEF2NED.dot(vel_ECEF)
    vel_decent = vel_NED[2]
    
    g_NED = np.array([0.0, 0.0, env.gravity(altitude)])
    Ta, Pa, rho, Cs = env.std_atmo(altitude)
    if rocket.timer_mode:
        if t > rocket.t_2nd_max or (altitude <= rocket.alt_sepa2 and t > rocket.t_2nd_min):
            CdS = rocket.CdS1 + rocket.CdS2
        else:
            CdS = rocket.CdS1
    else:
        CdS = rocket.CdS1 + rocket.CdS2 if altitude <= rocket.alt_sepa2 else rocket.CdS1
    
    drag_NED = np.array([0, 0, -0.5 * rho * vel_decent * np.abs(vel_decent) * CdS])
    acc_NED = drag_NED / rocket.m_burnout + g_NED
    acc_ECI = DCM_ECI2ECEF.transpose().dot(DCM_ECEF2NED.transpose().dot(acc_NED))

    wind_NED = env.Wind_NED(rocket.wind_speed(altitude), rocket.wind_direction(altitude))    
    vel_NED = vel_NED + wind_NED
    vel_ECI = DCM_ECI2ECEF.transpose().dot(DCM_ECEF2NED.transpose().dot(vel_NED))
    
    dx = np.zeros(6)
    dx[0:3] = vel_ECI
    dx[3:6] = acc_ECI

    return dx
