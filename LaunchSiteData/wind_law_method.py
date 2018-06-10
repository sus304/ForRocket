import numpy as np

def wind_law(alt_list, ref_wind_speed, ref_altitude, power_exp):
    speed_list = ref_wind_speed * (alt_list / ref_altitude) ** (1.0 / power_exp)
    dirction_list = np.zeros_like(alt_list)
    np.savetxt('wind.csv', np.c_[alt_list, speed_list, dirction_list], delimiter=',', fmt='%0.4f')
