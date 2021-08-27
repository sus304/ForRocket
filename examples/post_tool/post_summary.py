import numpy as np
import pandas as pd

def post_summary(df_all, file_prefix):
    vel_b_x_log = df_all["Vx-body [m/s]"]
    vel_b_y_log = df_all["Vy-body [m/s]"]
    vel_b_z_log = df_all["Vz-body [m/s]"]
    vel_norm_log = np.sqrt(vel_b_x_log ** 2 + vel_b_y_log ** 2 + vel_b_z_log ** 2)

    # Launch Clear
    fz_gravity_log = df_all["Fz-gravity [N]"]
    index_launch_clear = np.argmax(np.abs(fz_gravity_log) >= 0.1)
    time_launch_clear = df_all["Time [s]"][index_launch_clear]
    acc_launch_clear = df_all["Gccx-body [G]"][index_launch_clear]
    vel_launch_clear = vel_norm_log[index_launch_clear]
    aoa_launch_clear = df_all["AoA [deg]"][index_launch_clear]
    aos_launch_clear = df_all["AoS [deg]"][index_launch_clear]

    # Apogee
    index_apogee = np.argmax(df_all["Altitude [m]"])
    time_apogee = df_all["Time [s]"][index_apogee]
    altitude_apogee = df_all["Altitude [m]"][index_apogee]
    downrange_apogee = df_all["Downrange [m]"][index_apogee]
    vel_apogee = vel_norm_log[index_apogee]
    pos_apogee = [df_all["Latitude [deg]"][index_apogee], df_all["Longitude [deg]"][index_apogee]]

    # MaxQ
    index_maxq = np.argmax(df_all["DynamicPressure [kPa]"][:index_apogee])
    time_maxq = df_all["Time [s]"][index_maxq]
    altitude_maxq = df_all["Altitude [m]"][index_maxq]
    vel_maxq = vel_norm_log[index_maxq]
    mach_maxq = df_all["MachNumber [-]"][index_maxq]
    dynamics_pressure_maxq = df_all["DynamicPressure [kPa]"][index_maxq]

    # Max Vel
    index_maxvel = np.argmax(df_all["DynamicPressure [kPa]"][:index_apogee])
    time_maxvel = df_all["Time [s]"][index_maxvel]
    altitude_maxvel = df_all["Altitude [m]"][index_maxvel]
    vel_maxvel = vel_norm_log[index_maxvel]
    mach_maxvel = df_all["MachNumber [-]"][index_maxvel]
    dynamics_pressure_maxvel = df_all["DynamicPressure [kPa]"][index_maxvel]

    # Max Mach
    index_maxmach = np.argmax(df_all["DynamicPressure [kPa]"][:index_apogee])
    time_maxmach = df_all["Time [s]"][index_maxmach]
    altitude_maxmach = df_all["Altitude [m]"][index_maxmach]
    vel_maxmach = vel_norm_log[index_maxmach]
    mach_maxmach = df_all["MachNumber [-]"][index_maxmach]
    dynamics_pressure_maxmach = df_all["DynamicPressure [kPa]"][index_maxmach]

    # landing
    time_landing = np.array(df_all["Time [s]"])[-1]
    downrange_landing = np.array(df_all["Downrange [m]"])[-1]
    pos_landing = [np.array(df_all["Latitude [deg]"])[-1], np.array(df_all["Longitude [deg]"])[-1]]

    txt = open(file_prefix + '_summary.txt', mode='w')
    txt.writelines(['Launcher Clear X+,', str(round(time_launch_clear, 3)), '[s]\n'])
    txt.writelines(['Launcher Clear Acceleration,', str(round(acc_launch_clear, 3)), '[G]\n'])
    txt.writelines(['Launcher Clear Velocity,', str(round(vel_launch_clear, 3)), '[m/s]\n'])
    txt.writelines(['Launcher Clear AoA,', str(round(aoa_launch_clear, 3)), '[deg]\n'])
    txt.writelines(['Launcher Clear AoS,', str(round(aos_launch_clear, 3)), '[deg]\n'])
    txt.writelines(['Max Q X+,', str(round(time_maxq, 3)), '[s]\n'])
    txt.writelines(['Max Q Altitude,', str(round(altitude_maxq, 3)), '[m]\n'])
    txt.writelines(['Max Q Velocity,', str(round(vel_maxq, 3)), '[m/s]\n'])
    txt.writelines(['Max Q MachNumber,', str(round(mach_maxq, 3)), '[-]\n'])
    txt.writelines(['Max Q Dynamic Pressure,', str(round(dynamics_pressure_maxq, 3)), '[kPa]\n'])
    txt.writelines(['Max Speed X+,', str(round(time_maxvel, 3)), '[s]\n'])
    txt.writelines(['Max Speed Altitude,', str(round(altitude_maxvel, 3)), '[m]\n'])
    txt.writelines(['Max Speed Velocity,', str(round(vel_maxvel, 3)), '[m/s]\n'])
    txt.writelines(['Max Speed MachNumber,', str(round(mach_maxvel, 3)), '[-]\n'])
    txt.writelines(['Max Speed Dynamic Pressure,', str(round(dynamics_pressure_maxvel, 3)), '[kPa]\n'])
    txt.writelines(['Max Mach Number X+,', str(round(time_maxmach, 3)), '[s]\n'])
    txt.writelines(['Max Mach Number Altitude,', str(round(altitude_maxmach, 3)), '[m]\n'])
    txt.writelines(['Max Mach Velocity,', str(round(vel_maxmach, 3)), '[m/s]\n'])
    txt.writelines(['Max Mach Number,', str(round(mach_maxmach, 3)), '[-]\n'])
    txt.writelines(['Max Mach Dynamic Pressure,', str(round(dynamics_pressure_maxmach, 3)), '[kPa]\n'])
    txt.writelines(['Apogee X+,', str(round(time_apogee, 3)), '[s]\n'])
    txt.writelines(['Apogee Altitude,', str(round(altitude_apogee, 3)), '[m]\n'])
    txt.writelines(['Apogee Downrange,', str(round(downrange_apogee, 3)), '[m]\n'])
    txt.writelines(['Apogee Air Velocity,', str(round(vel_apogee, 3)), '[m/s]\n'])
    txt.writelines(['Apogee Point,', str(pos_apogee), '\n'])
    txt.writelines(['Landing X+,', str(round(time_landing, 3)), '[s]\n'])
    txt.writelines(['Landing Downrange,', str(round(downrange_landing, 3)), '[m]\n'])
    txt.writelines(['Landing Point,', str(pos_landing), '\n'])
    txt.close()


    df = pd.DataFrame({'lat': [pos_landing[0]],
                        'lon': [pos_landing[1]]},
    )
    df.to_csv(file_prefix + '_landing_point.csv', index=False)

    return txt, pos_landing