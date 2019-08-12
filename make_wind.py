import numpy as np
import matplotlib.pyplot as plt

########## USER INPUT ###########
height_wind_reference = 5.0  # [m]
wind_velocity = 2.5  # [m/s]
wind_direction = 45.0  # [deg] north clockwise
wind_exponatial = 6.0

altitude_max = 200.0  # [m]
#################################


def make_law_wind(height_ref, vel_ref, dir_ref, exp_a, alt_max):
    def law_method(alt):
        return vel_ref * (alt / height_ref) ** (1 / exp_a)

    base = np.cos(np.arange(0.0, 0.5*np.pi, 0.005))
    alt_array = alt_max * (1 - base)
    vel_array = law_method(alt_array)
    dir_array = np.zeros_like(alt_array) + dir_ref

    return alt_array, vel_array, dir_array



if __name__ == '__main__':
    alt_array, vel_array, dir_array = make_law_wind(height_wind_reference, wind_velocity, wind_direction, wind_exponatial, altitude_max, altitude_step)
    np.savetxt('wind.csv', np.c_[alt_array, vel_array, dir_array], delimiter=',', fmt='%0.4f', comments='', header='alt,vel,dir')
