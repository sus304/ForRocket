import os
import sys
import datetime
import json
import numpy as np
import pandas as pd
from scipy import interpolate
from tqdm import tqdm
import simplekml
from Simulator.rocket_param import Rocket
import Simulator.solver as solver
from make_wind import make_law_wind

## 風の形はconfigから入力されたwind.csvから取得する
## wind.csvの風速をリファレンス高度での風速を基にスケーリングしてスカラ風速を用意
## 風向は一様とする
########## USER INPUT ###########
wind_direction_min = 0.0  # [deg]  north clockwise
wind_direction_max = 315.0  # [deg]  north clockwise
wind_direction_step = 45.0  # [deg]  north clockwise

wind_model = 'original'  # original or law

## law wind model parameter
wind_velocity_min = 1.0  # [m/s]
wind_velocity_max = 7.0  # [m/s]
wind_velocity_step = 1.0  # [m/s]

height_wind_reference = 5.0  # [m]
wind_exponatial = 6.0

#################################


def __original_wind_solver(wind_dir, wind_original, json_config, json_engine, result_dir):
    # 各条件でのリザルトフォルダ作成
    result_dir_single = '/Result_single_dir' + str(wind_dir)
    os.mkdir(result_dir + result_dir_single)

    # 計算用風データ作成
    wind_alt_single_array = wind_original[:, 0]
    wind_vel_single_array = wind_original[:, 1]
    wind_dir_single_array = np.zeros_like(wind_alt_single_array) + wind_dir
    np.savetxt(result_dir + result_dir_single + '/wind.csv', np.c_[wind_alt_single_array, wind_vel_single_array, wind_dir_single_array], delimiter=',', header='alt[m],vel[m/s],dir[deg]',fmt='%0.4f', comments='')
    json_config['Wind']['Wind File'] = result_dir + result_dir_single + '/wind.csv'

    rocket = Rocket(json_config, json_engine, result_dir + result_dir_single)
    solver.solve_trajectory(rocket)
    # rocket.result.output_min(rocket)
    rocket.result.output_full(rocket)
    return rocket

def __law_wind_solver(wind_vel, wind_dir, json_config, json_engine, result_dir):
    # 各条件でのリザルトフォルダ作成
    result_dir_single = '/Result_single_vel' + str(wind_vel) + '_dir' + str(wind_dir)
    os.mkdir(result_dir + result_dir_single)

    # 計算用風データ作成
    wind_alt_single_array, wind_vel_single_array, wind_dir_single_array = make_law_wind(height_wind_reference, wind_vel, wind_dir, wind_exponatial, 10000.0)
    np.savetxt(result_dir + result_dir_single + '/wind.csv', np.c_[wind_alt_single_array, wind_vel_single_array, wind_dir_single_array], delimiter=',', header='alt[m],vel[m/s],dir[deg]',fmt='%0.4f', comments='')
    json_config['Wind']['Wind File'] = result_dir + result_dir_single + '/wind.csv'

    rocket = Rocket(json_config, json_engine, result_dir + result_dir_single)
    solver.solve_trajectory(rocket)
    # rocket.result.output_min(rocket)
    rocket.result.output_full(rocket)
    return rocket


def multi_solver(config_file):
    time_start_calc = datetime.datetime.now()

    json_config = json.load(open(config_file))
    json_engine = json.load(open(json_config.get('System').get('Engine Config json')))
    model_name = json_config.get('System').get('Name')

    print('Config File: ', config_file)
    print('Config File: ', json_config.get('System').get('Engine Config json'))
    print('Model: ', model_name)

    # Make Result Directory
    result_dir = './Result_multi_' + model_name
    if os.path.exists(result_dir):
        resultdir_org = result_dir
        i = 1
        while os.path.exists(result_dir):
            result_dir = resultdir_org + '_%02d' % (i)
            i = i + 1
    os.mkdir(result_dir)

    wind_dir_array = np.arange(wind_direction_min, wind_direction_max + wind_direction_step, wind_direction_step)
    if wind_model == 'law':
        wind_vel_array = np.arange(wind_velocity_min, wind_velocity_max + wind_velocity_step, wind_velocity_step)
    elif wind_model == 'original':
        try:
            wind_file = json_config.get('Wind').get('Wind File')
        except:
            print('Not Found Wind File')
            sys.exit()
        wind_original = np.loadtxt(wind_file, delimiter=',', skiprows=1)
        wind_vel_array = np.array([1])

    time_apogee_table = np.zeros((len(wind_vel_array), len(wind_dir_array)))
    vel_apogee_table = np.zeros_like(time_apogee_table)
    altitude_apogee_table = np.zeros_like(time_apogee_table)
    downrange_hard_table = np.zeros_like(time_apogee_table)
    downrange_soft_table = np.zeros_like(time_apogee_table)
    vel_max_table = np.zeros_like(time_apogee_table)
    mach_max_table = np.zeros_like(time_apogee_table)
    maxQ_table = np.zeros_like(time_apogee_table)
    time_hard_landing_table = np.zeros_like(time_apogee_table)
    time_sepa2_table = np.zeros_like(time_apogee_table)
    time_soft_landing_table = np.zeros_like(time_apogee_table)
    hard_landing_points = [[0 for y in range(len(wind_dir_array))] for x in range(len(wind_vel_array))]
    soft_landing_points = [[0 for y in range(len(wind_dir_array))] for x in range(len(wind_vel_array))]

    for i_vel in tqdm(range(len(wind_vel_array))):
        for j_dir in range(len(wind_dir_array)):
            if wind_model == 'law':
                single = __law_wind_solver(wind_vel_array[i_vel], wind_dir_array[j_dir], json_config, json_engine, result_dir)
            elif wind_model == 'original':
                single = __original_wind_solver(wind_dir_array[j_dir], wind_original, json_config, json_engine, result_dir)

            time_apogee_table[i_vel][j_dir] = single.result.time_apogee
            vel_apogee_table[i_vel][j_dir] = single.result.vel_apogee
            altitude_apogee_table[i_vel][j_dir] = single.result.alt_apogee
            downrange_hard_table[i_vel][j_dir] = single.result.downrange_hard_landing
            downrange_soft_table[i_vel][j_dir] = single.result.downrange_soft_landing
            vel_max_table[i_vel][j_dir] = single.result.vel_max
            mach_max_table[i_vel][j_dir] = single.result.Mach_max
            maxQ_table[i_vel][j_dir] = single.result.maxQ
            time_hard_landing_table[i_vel][j_dir] = single.result.time_hard_landing
            time_sepa2_table[i_vel][j_dir] = single.result.time_separation_2nd
            time_soft_landing_table[i_vel][j_dir] = single.result.time_soft_landing
            hard_landing_points[i_vel][j_dir] = list(single.result.pos_hard_LLH_log[-1, :])
            soft_landing_points[i_vel][j_dir] = list(single.result.pos_soft_LLH_log[-1, :])

    table_index = (wind_vel_array)
    table_header = (wind_dir_array)
    pd.DataFrame(time_apogee_table, index=table_index, columns=table_header).to_csv(result_dir + '/time_apogee.csv')
    pd.DataFrame(vel_apogee_table, index=table_index, columns=table_header).to_csv(result_dir + '/vel_apogee.csv')
    pd.DataFrame(altitude_apogee_table, index=table_index, columns=table_header).to_csv(result_dir + '/altitude_apogee.csv')
    pd.DataFrame(downrange_hard_table, index=table_index, columns=table_header).to_csv(result_dir + '/downrange_hard.csv')
    pd.DataFrame(downrange_soft_table, index=table_index, columns=table_header).to_csv(result_dir + '/downrange_soft.csv')
    pd.DataFrame(vel_max_table, index=table_index, columns=table_header).to_csv(result_dir + '/vel_max.csv')
    pd.DataFrame(mach_max_table, index=table_index, columns=table_header).to_csv(result_dir + '/mach_max.csv')
    pd.DataFrame(maxQ_table, index=table_index, columns=table_header).to_csv(result_dir + '/maxQ.csv')
    pd.DataFrame(time_hard_landing_table, index=table_index, columns=table_header).to_csv(result_dir + '/time_hard_landing.csv')
    pd.DataFrame(time_sepa2_table, index=table_index, columns=table_header).to_csv(result_dir + '/time_sepa_2nd.csv')
    pd.DataFrame(time_soft_landing_table, index=table_index, columns=table_header).to_csv(result_dir + '/time_soft_landing.csv')
    
    pd.DataFrame(hard_landing_points, index=table_index, columns=table_header).to_csv(result_dir + '/posLLH_hard.csv')
    pd.DataFrame(soft_landing_points, index=table_index, columns=table_header).to_csv(result_dir + '/posLLH_soft.csv')

    def __plot_kml(landing_points_LLH, name=''):
        kml = simplekml.Kml()
        for vel_iter in landing_points_LLH:
            points = []
            linestring = kml.newlinestring()
            linestring.style.linestyle.color = simplekml.Color.orange
            for i in range(len(vel_iter)):
                lat = vel_iter[i][0]
                lon = vel_iter[i][1]
                points.append([lon, lat, 0.0])
            points.append(points[0])
            linestring.coords = points
        kml.save(result_dir + '/landing_range_' + name + '.kml')

    __plot_kml(hard_landing_points, name='hard')
    __plot_kml(soft_landing_points, name='soft')

    print('Completed solve!')
    print('Output Result Directory: ', result_dir)    

    time_end_calc = datetime.datetime.now()
    print('Calculate Time: ', time_end_calc - time_start_calc)



if __name__ == '__main__':
    print('Welcome to Multi Solver of ForRocket!')
    print('Start Time: ', datetime.datetime.now(), 'UTC')

    argv = sys.argv
    if len(argv) < 2:
        print('Error!! config file is missing')
        print('Usage: python multi_solver.py configFileName.json')
        sys.exit()  

    arg_file = argv[1]
    if '.json' not in arg_file:
        print('Error!! Second arg is config file name with ".json" extention')
        sys.exit()

    multi_solver(arg_file)

    print('End Time: ', datetime.datetime.now(), 'UTC')
