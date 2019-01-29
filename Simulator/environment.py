import numpy as np

# ref. 1976 standard atmosphere
# ジオポテンシャル高度を基準として標準大気の各層の気温減率から各大気値を算出
# 高度86 kmまで対応
def std_atmo(altitude):
    # altitude [m]
    R = 287.1
    gamma = 1.4
    Re = 6378.137e3 # Earth Radius [m]
    g0 = 9.80665

    # atmospheric layer
    h_list  = [0.0, 11.0e3, 20.0e3, 32.0e3, 47.0e3, 51.0e3, 71.0e3, 84.852e3] # geopotential height [m]
    TG_list = [-6.5e-3, 0.0, 1.0e-3, 2.8e-3, 0, -2.8e-3, -2.0e-3, 0.0] # Temp. gradient [K/m]
    T_list  = [288.15, 216.65, 216.65, 228.65, 270.65, 270.65, 214.65, 186.946] # [K]
    P_list  = [101325.0, 22632.0, 5474.9, 868.02, 110.91, 66.939, 3.9564, 0.3734] # [Pa]

    h = altitude * Re / (Re + altitude) # geometric altitude => geopotential height

    k = 0 # dafault layer
    for i in range(8):
        if h < h_list[i]:
            k = i - 1
            if k < 0:
                k = 0
            break
        elif h >= h_list[7]:
            k = 7
            break

    Temperature = T_list[k] + TG_list[k] * (h - h_list[k]) # [K]
    if TG_list[k] == 0.0:
        Pressure = P_list[k] * np.exp(g0 / R * (h_list[k] - h) / T_list[k])
    else:
        Pressure = P_list[k] * np.power(T_list[k] / Temperature, g0 / R / TG_list[k]) # [Pa]
    Density = Pressure / (R * Temperature) # [kg/m^3]
    SoundSpeed = np.sqrt(gamma * R * Temperature) # [m/s]

    return Temperature, Pressure, Density, SoundSpeed

def get_std_temp(altitude):
    return std_atmo(altitude)[0]

def get_std_temp_array(altitude_array):
    output = []
    for alt in altitude_array:
        output.append(get_std_temp(alt))
    return np.array(output)

def get_std_press(altitude):
    return std_atmo(altitude)[1]

def get_std_press_array(altitude_array):
    output = []
    for alt in altitude_array:
        output.append(get_std_press(alt))
    return np.array(output)

def get_std_density(altitude):
    return std_atmo(altitude)[2]

def get_std_density_array(altitude_array):
    output = []
    for alt in altitude_array:
        output.append(get_std_density(alt))
    return np.array(output)

def get_std_soundspeed(altitude):
    return std_atmo(altitude)[3]

def get_std_soundspeed_array(altitude_array):
    output = []
    for alt in altitude_array:
        output.append(get_std_soundspeed(alt))
    return np.array(output)

def gravity(altitude):
    # altitude [m]
    if altitude < 0.0:
        altitude = 0.0
    Re = 6378.137e3 # Earth Radius [m]
    g0 = 9.80665
    gravity = g0 * (Re / (Re + altitude)) ** 2 # [m/s^2]
    return gravity

def Wind_NED(WindSpeed, WindDirection):
    '''
    Input: WindSpeed [m/s], WindDirection [deg]
    '''
    # WindSpeed [m/s]
    # WindDirection [deg] 北から時計回り@NED
    # 負にすることで風向"からの"風にしてる

    Wind_NED = np.zeros(3)
    Wind_NED[0] = -WindSpeed * np.cos(np.radians(WindDirection))
    Wind_NED[1] = -WindSpeed * np.sin(np.radians(WindDirection))
    Wind_NED[2] = 0.0
    return Wind_NED

def magnetic_declination(lat, lon):
    # Ref. https://vldb.gsi.go.jp/sokuchi/geomag/menu_04/index.html 2019/01/27
    delta_lat = lat - 37.0
    delta_lon = lon - 138.0
    mag_dec = (7.0 + 57.201 / 60.0)	+ (18.750 / 60.0) * delta_lat - (6.761 / 60.0) * delta_lon - (0.059 / 60.0) * delta_lat ** 2 - (0.014 / 60.0) * delta_lat * delta_lon - (0.579 / 60.0) * delta_lon ** 2
    return mag_dec

if __name__ == '__main__':
    import time
    from scipy import interpolate
    import matplotlib.pyplot as plt
    alt_array = np.arange(0.0, 86*1000.0, 1.0)

    start = time.time()
    for i in range(1000000):
        a = get_std_press(1000.0)
    end = time.time()
    print(end - start, 'sec')

    press_array = get_std_press_array(alt_array)
    inter = interpolate.interp1d(alt_array, press_array, kind='linear', fill_value=(press_array[0], press_array[-1]))
    start = time.time()
    for i in range(1000000):
        a = inter(1000.0)
    end = time.time()
    print(end - start, 'sec')
    
        